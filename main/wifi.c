#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
//#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "routing_info.h"
#include "com.h"

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE];
static char key[MAX_SSID_SIZE];

const int WIFI_CONNECTED_BIT = BIT0;
const int WIFI_SOCKET_DISCONNECTED = BIT1;
static EventGroupHandle_t s_wifi_event_group;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int sock = -1;
/* Accepted WiFi connection */
static int conn = -1;

/* WiFi event handler */
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "got ip:%s",
                ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

      txp.src = MAKE_ROUTE(ESP32, WIFI_CTRL);
      txp.dst = MAKE_ROUTE(GAP8, WIFI_CTRL);
      txp.data[0] = 0x21; // WiFi connected
      memcpy(&txp.data[1], &event->event_info.got_ip.ip_info.ip.addr, sizeof(uint32_t));
      txp.length = 3 + sizeof(uint32_t);

      ESP_LOGI(TAG, "0x%04X", (uint32_t) event->event_info.got_ip.ip_info.ip.addr);

      // TODO: We should probably not block here...
      com_send_blocking(&txp);

      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      ESP_LOGI(TAG,"Disconnected from access point");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                MAC2STR(event->event_info.sta_connected.mac),
                event->event_info.sta_connected.aid);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                MAC2STR(event->event_info.sta_disconnected.mac),
                event->event_info.sta_disconnected.aid);
      break;
    default:
        break;
  }
  return ESP_OK;
}

static void wifi_init_sta(const char * ssid, const char * key)
{
  s_wifi_event_group = xEventGroupCreate();

  //tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void wifi_ctrl(void* _param) {
  uint8_t length;
  uint8_t count;

  while (1) {
    com_receive_wifi_ctrl_blocking((esp_routable_packet_t*) &rxp);

    switch (rxp.data[0]) {
      case 0x10: // key
        ESP_LOGI("WIFI", "Should set SSID");
        memcpy(ssid, &rxp.data[1], rxp.length - 3);
        ssid[rxp.length - 3 + 1] = 0;
        ESP_LOGD(TAG, "SSID: %s", ssid);
        // Save to NVS?
        break;
      case 0x11: // key
        ESP_LOGI("WIFI", "Should set password");
        memcpy(key, &rxp.data[1], rxp.length - 3);
        key[rxp.length - 3 + 1] = 0;
        ESP_LOGD(TAG, "KEY: %s", key);
        // Save to NVS?
        break;
      case 0x20: // Connect
        ESP_LOGD("WIFI", "Should connect");
        wifi_init_sta(ssid, key);
        break;
    }
  }    
}

void wifi_bind_socket() {
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(5000);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket created");

  int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket binded");

  err = listen(sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket listening");
}

void wifi_wait_for_socket_connected() {
  ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  uint addrLen = sizeof(sourceAddr);
  conn = accept(sock, (struct sockaddr *)&sourceAddr, &addrLen);
  if (conn < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
  }
  ESP_LOGI(TAG, "Connection accepted");
}

void wifi_wait_for_disconnect() {
  xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
}

static void wifi_task(void *pvParameters) {
  wifi_bind_socket();
  while (1) {
    //blink_period_ms = 500;
    wifi_wait_for_socket_connected();
    //blink_period_ms = 100;

    // Not thread safe!
    txp.src = MAKE_ROUTE(ESP32, WIFI_CTRL);
    txp.dst = MAKE_ROUTE(GAP8, WIFI_CTRL);
    txp.data[0] = 0x23; // WiFi client connection status
    txp.data[1] = 1;
    txp.length = 4;
    com_send_blocking(&txp);

    // Probably not the best, should be handled in some other way?
    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");
  }
}

void wifi_send_packet(const char * buffer, size_t size) {
  if (conn != -1) {
    ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
    int err = send(conn, buffer, size, 0);
    if (err < 0) {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      conn = -1;
      xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
    }
  } else {
    ESP_LOGE(TAG, "No socket when trying to send data");
    xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
  }
}

/*void wifi_router_post_packet(esp_packet_t * packet) {
  // Do we queue packet here instead?
  wifi_send_packet(packet->data, packet->len);
}*/

static esp_routable_packet_t txp_wifi;
static void wifi_sending_task(void *pvParameters) {
  while (1) {
    com_receive_wifi_data_blocking(&txp_wifi);
    wifi_send_packet(&txp_wifi, txp_wifi.length + 2);
  }
}

static esp_routable_packet_t rxp_wifi;
static void wifi_receiving_task(void *pvParameters) {
  int len;
  while (1) {
    // Lock on no client connected!
    len = recv(conn, &rxp_wifi, 1, 0);
    //ESP_LOGI(TAG, "Data len %i", len);
    if (len > 0) {
      ESP_LOGI(TAG, "Data len read %i", len);
      len = recv(conn, &rxp_wifi.dst, rxp_wifi.length, 0);
      ESP_LOGI(TAG, "Data len %i", len);
      com_send_blocking(&rxp_wifi);  
    } else {
      vTaskDelay(10);
    }
  }
}


void wifi_init() {
  tcpip_adapter_init();

  s_wifi_event_group = xEventGroupCreate();

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 10000, NULL, 1, NULL);

  xTaskCreate(wifi_task, "WiFi TASk", 10000, NULL, 1, NULL);

  xTaskCreate(wifi_sending_task, "WiFi TX", 10000, NULL, 1, NULL);

  xTaskCreate(wifi_receiving_task, "WiFi RX", 10000, NULL, 1, NULL);  


  ESP_LOGI("WIFI", "Wifi initialized");
}