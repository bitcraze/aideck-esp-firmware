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

  tcpip_adapter_init();
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

void wifi_init() {
  xTaskCreate(wifi_ctrl, "WiFi CTRL", 10000, NULL, 1, NULL);

  ESP_LOGI("WIFI", "Wifi initialized");
}