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

#include "com.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "router.h"
#include "esp_transport.h"


#define ESP_WIFI_CTRL_QUEUE_LENGTH (2)
#define ESP_WIFI_CTRL_QUEUE_SIZE (sizeof(esp_routable_packet_t))
#define ESP_SYS_QUEUE_LENGTH (2)
#define ESP_SYS_QUEUE_SIZE (sizeof(esp_routable_packet_t))
#define ESP_TEST_QUEUE_LENGTH (2)
#define ESP_TEST_QUEUE_SIZE (sizeof(esp_routable_packet_t))

static xQueueHandle espWiFiCTRLQueue;
static xQueueHandle espSystemQueue;
static xQueueHandle espTESTQueue;

static esp_routable_packet_t rxp;

static const int START_UP_RX_TASK = BIT0;
static EventGroupHandle_t startUpEventGroup;

static void com_rx(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1) {
    ESP_LOGD("COM", "Waiting for packet");
    espAppReceiveFromRouter(&rxp);
    ESP_LOGD("COM", "Received packet for 0x%02X", rxp.route.destination);
    ESP_LOG_BUFFER_HEX_LEVEL("COM", &rxp, 10, ESP_LOG_DEBUG);
    switch (rxp.route.function) {
      case CPX_F_TEST:
        xQueueSend(espTESTQueue, &rxp, (TickType_t) portMAX_DELAY);
        break;
      case CPX_F_WIFI_CTRL:
        xQueueSend(espWiFiCTRLQueue, &rxp, (TickType_t) portMAX_DELAY);
        break; 
      case CPX_F_SYSTEM:
        xQueueSend(espSystemQueue, &rxp, (TickType_t) portMAX_DELAY);
        break;
      default:
        ESP_LOGW("COM", "Cannot handle 0x%02X", rxp.route.function);
    }
  }
}

void com_init() {
  espWiFiCTRLQueue = xQueueCreate(ESP_WIFI_CTRL_QUEUE_LENGTH, ESP_WIFI_CTRL_QUEUE_SIZE);
  espSystemQueue = xQueueCreate(ESP_SYS_QUEUE_LENGTH, ESP_SYS_QUEUE_SIZE);
  espTESTQueue = xQueueCreate(ESP_TEST_QUEUE_LENGTH, ESP_TEST_QUEUE_SIZE);

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_RX_TASK);
  xTaskCreate(com_rx, "COM RX", 5000, NULL, 1, NULL);
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_RX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  ESP_LOGI("COM", "Initialized");
}

void com_receive_test_blocking(esp_routable_packet_t * packet) {
  xQueueReceive(espTESTQueue, packet, portMAX_DELAY);
}

void com_receive_wifi_ctrl_blocking(esp_routable_packet_t * packet) {
  xQueueReceive(espWiFiCTRLQueue, packet, portMAX_DELAY);
}

void com_receive_system_blocking(esp_routable_packet_t * packet) {
  xQueueReceive(espSystemQueue, packet, (TickType_t) portMAX_DELAY);
}
