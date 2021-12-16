#include "test.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "com.h"

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

//#define RUN_TESTS_FROM_ESP32

#ifdef RUN_TESTS_FROM_ESP32
static void test_runner(void* _param) {
  while (1) {
    
}
#endif

static void test_rx(void* _param) {
  uint8_t length;
  uint8_t count;

  while (1) {
    com_receive_test_blocking((esp_routable_packet_t*) &rxp);
    switch(rxp.route.source) {
      case 0x0: ESP_LOGD("TEST", "Request from STM32"); break;
      case 0x2: ESP_LOGD("TEST", "Request from GAP8"); break;
      default: ESP_LOGW("TEST", "Request from UNKNOWN"); break;
    }

    switch (rxp.data[0]) {
      case 0:
        // Sink, do nothing
        ESP_LOGD("TEST", "SINK");
        break;
      case 1:
        // echo
        ESP_LOGD("TEST", "ECHO");
        //printf("Got packet of length %u\n", rxp.length);
        memcpy(&txp, &rxp, rxp.length + 4);
        txp.route.source = rxp.route.destination;
        txp.route.destination = rxp.route.source;
        //printf("Will send packet of length %u\n", txp.length);
        com_send_blocking((esp_routable_packet_t*) &txp);
        break;
      case 2:
        // Source
        length = rxp.data[2];
        count = rxp.data[1];
        txp.length = length;
        txp.route.source = rxp.route.destination;
        txp.route.destination = rxp.route.source;

        ESP_LOGD("TEST", "SOURCE %u packets of size %u\n", count, length);

        for (int i = 0; i < count; i++) {
          for (int j = 0; j < length; j++) {
            txp.data[j] = j;
          }
          com_send_blocking((esp_routable_packet_t*) &txp);
        }
        break;
      default:
         ESP_LOGW("TEST", "UNKNOWN");
    }
  }    
}

void test_init() {
  xTaskCreate(test_rx, "TEST RX", 10000, NULL, 1, NULL);

#ifdef RUN_TESTS_FROM_ESP32  
  xTaskCreate(test_runner, "TEST runner", 10000, NULL, 1, NULL);
  ESP_LOGI("TEST", "Test runner initialized, will run tests");
#endif

  ESP_LOGI("TEST", "Test RX initialized");
}