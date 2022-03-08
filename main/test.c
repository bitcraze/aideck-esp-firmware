/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

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
    com_receive_test_blocking(&rxp);
    switch(rxp.route.source) {
      case CPX_T_STM32: ESP_LOGD("TEST", "Request from STM32"); break;
      case CPX_T_GAP8: ESP_LOGD("TEST", "Request from GAP8"); break;
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
        memcpy(&txp, &rxp, rxp.dataLength + 4);
        txp.route.source = rxp.route.destination;
        txp.route.destination = rxp.route.source;
        //printf("Will send packet of length %u\n", txp.length);
        espAppSendToRouterBlocking((esp_routable_packet_t*) &txp);
        break;
      case 2:
        // Source
        length = rxp.data[2];
        count = rxp.data[1];
        txp.dataLength = length;
        txp.route.source = rxp.route.destination;
        txp.route.destination = rxp.route.source;

        ESP_LOGD("TEST", "SOURCE %u packets of size %u\n", count, length);

        for (int i = 0; i < count; i++) {
          for (int j = 0; j < length; j++) {
            txp.data[j] = j;
          }
          espAppSendToRouterBlocking((esp_routable_packet_t*) &txp);
        }
        break;
      default:
         ESP_LOGW("TEST", "UNKNOWN");
    }
  }
}

void test_init() {
  xTaskCreate(test_rx, "TEST RX", 5000, NULL, 1, NULL);

#ifdef RUN_TESTS_FROM_ESP32
  xTaskCreate(test_runner, "TEST runner", 5000, NULL, 1, NULL);
  ESP_LOGI("TEST", "Test runner initialized, will run tests");
#endif

  ESP_LOGI("TEST", "Test RX initialized");
}
