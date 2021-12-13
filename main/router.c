#include "router.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "routing_info.h"
#include "spi_transport.h"
#include "uart_transport.h"
#include "com.h"

static spi_transport_packet_t gap8_rxp;
static spi_transport_packet_t gap8_txp;
static uart_transport_packet_t cf_rxp;
static uart_transport_packet_t cf_txp;

static esp_packet_t esp_rxp;
static esp_packet_t esp_txp;

static esp_packet_t esp_gap8_txp;

static void router_from_gap8(void* _param) {

    while(1) {
      // Wait for incoming packet from GAP8
      spi_transport_receive(&gap8_rxp);

      spi_transport_routable_packet_t * routable = (spi_transport_routable_packet_t*) &gap8_rxp;

      switch ((routable->dst >> 4)) {
        case CF_TARGET:
          ESP_LOGD("ROUTER", "GAP8 [0x%02X] -> CF [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // We need to split up the package here!
          cf_txp.length = gap8_rxp.length;
          memcpy(cf_txp.data, gap8_rxp.data, cf_txp.length);
          uart_transport_send(&cf_txp);
          break;
        case ESP32_TARGET:
          ESP_LOGD("ROUTER", "GAP8 [0x%02X] -> ESP32 [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // We need to split up the package here!
          esp_gap8_txp.length = gap8_rxp.length;
          memcpy(esp_gap8_txp.data, gap8_rxp.data, esp_gap8_txp.length);
          com_router_post_packet(&esp_gap8_txp);
          break;          
        default:
          ESP_LOGW("ROUTER", "Cannot from GAP8 to [0x%02X]", (routable->dst >> 4));
      }
    }
}

static void router_from_crazyflie(void* _param) {

    while(1) {
      uart_transport_receive(&cf_rxp);

      uart_transport_routable_packet_t * routable = (uart_transport_routable_packet_t*) &cf_rxp;

      switch(routable->dst >> 4) {
        case GAP8_TARGET:
          ESP_LOGD("ROUTER", "CF [0x%02X] -> GAP8 [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // Right now we're always sending to GAP8 (which always has higher MTU)
          gap8_txp.length = cf_rxp.length;
          memcpy(gap8_txp.data, cf_rxp.data, cf_rxp.length);
          spi_transport_send(&gap8_txp);
          break;
        case ESP32_TARGET:
          ESP_LOGD("ROUTER", "CF [0x%02X] -> ESP32 [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // Right now we're always sending to ESP32 (which always has higher MTU)
          esp_txp.length = cf_rxp.length;
          memcpy(esp_txp.data, cf_rxp.data, cf_rxp.length);
          com_router_post_packet(&esp_txp);
          break;
        default:
          ESP_LOGW("ROUTER", "Cannot route from CF to [0x%02X]", (routable->dst >> 4));

      }
    }
}

static void router_from_esp32(void* _param) {

    while(1) {
      com_router_get_packet(&esp_rxp);

      esp_routable_packet_t * routable = (esp_routable_packet_t*) &esp_rxp;

      switch(routable->dst >> 4) {
        case GAP8_TARGET:
          ESP_LOGD("ROUTER", "ESP32 [0x%02X] -> GAP8 [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // Right now we're always sending to GAP8 (which always has higher MTU)
          gap8_txp.length = esp_rxp.length;
          memcpy(gap8_txp.data, esp_rxp.data, esp_rxp.length);
          spi_transport_send(&gap8_txp);
          break;
        case CF_TARGET:
          ESP_LOGD("ROUTER", "ESP32 [0x%02X] -> CF [0x%02X] (%u)", routable->src, routable->dst, routable->length);
          // We need to split up the package here!
          cf_txp.length = esp_rxp.length;
          memcpy(cf_txp.data, esp_rxp.data, cf_txp.length);
          uart_transport_send(&cf_txp);
          break;
        default:
          ESP_LOGW("ROUTER", "Cannot route from ESP32 to [0x%02X]", (routable->dst >> 4));

      }
    }
}

void router_init() {
  xTaskCreate(router_from_gap8, "Router from GAP8", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_crazyflie, "Router from CF", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_esp32, "Router from ESP32", 10000, NULL, 1, NULL);

  ESP_LOGI("ROUTER", "Initialized");
}