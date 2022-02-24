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

#include "router.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "cpx.h"
#include "spi_transport.h"
#include "uart_transport.h"
#include "esp_transport.h"
#include "wifi.h"

typedef struct {
  CPXRoutablePacket_t txp;
} RouteContext_t;

static RouteContext_t wifi_task_context;
static CPXRoutablePacket_t wifiRxBuf;

static RouteContext_t gap8_task_context;
static CPXRoutablePacket_t spiRxBuf;

static RouteContext_t cf_task_context;
static CPXRoutablePacket_t uartRxBuf;

static RouteContext_t esp_task_context;
static CPXRoutablePacket_t espRxBuf;

typedef void (*Receiver_t)(CPXRoutablePacket_t* packet);
typedef void (*Sender_t)(const CPXRoutablePacket_t* packet);

static const int START_UP_GAP8_ROUTER_RUNNING = BIT0;
static const int START_UP_CF_ROUTER_RUNNING = BIT1;
static const int START_UP_ESP_ROUTER_RUNNING = BIT2;
static const int START_UP_WIFI_ROUTER_RUNNING = BIT3;
static EventGroupHandle_t startUpEventGroup;

static void splitAndSend(const CPXRoutablePacket_t* rxp, RouteContext_t* context, Sender_t sender) {
  CPXRoutablePacket_t* txp = &context->txp;

  // TODO krri Split packet if needed
  txp->route = rxp->route;
  memcpy(txp->data, rxp->data, rxp->dataLength);

  txp->dataLength = rxp->dataLength;
  sender(txp);
}

static void route(Receiver_t receive, CPXRoutablePacket_t* rxp, RouteContext_t* context, const char* routerName) {
  while(1) {
    receive(rxp);

    const uint8_t source = rxp->route.source;
    const uint8_t destination = rxp->route.destination;
    const uint16_t cpxDataLength = rxp->dataLength;

    switch (destination) {
      case GAP8:
        ESP_LOGD("ROUTER", "%s [0x%02X] -> GAP8 [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
        splitAndSend(rxp, context, spi_transport_send);
        break;
      case STM32:
        ESP_LOGD("ROUTER", "%s [0x%02X] -> STM32 [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
        splitAndSend(rxp, context, uart_transport_send);
        break;
      case ESP32:
        ESP_LOGD("ROUTER", "%s [0x%02X] -> ESP32 [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
        splitAndSend(rxp, context, espTransportSend);
        break;
      case HOST:
        ESP_LOGD("ROUTER", "%s [0x%02X] -> HOST [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
        splitAndSend(rxp, context, wifi_transport_send);
        break;
      default:
        ESP_LOGW("ROUTER", "Cannot route from %s [0x%02X] to [0x%02X]", routerName, source, destination);
    }
  }
}

static void router_from_gap8(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_GAP8_ROUTER_RUNNING);
  route(spi_transport_receive, &spiRxBuf, &gap8_task_context, "GAP8");
}

static void router_from_crazyflie(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_CF_ROUTER_RUNNING);
  route(uart_transport_receive, &uartRxBuf, &cf_task_context, "STM32");
}

static void router_from_esp32(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_ESP_ROUTER_RUNNING);
  route(espTransportReceive, &espRxBuf, &esp_task_context, "ESP32");
}

static void router_from_wifi(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_WIFI_ROUTER_RUNNING);
  route(wifi_transport_receive, &wifiRxBuf, &wifi_task_context, "HOST");
}


void router_init() {
  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_GAP8_ROUTER_RUNNING | START_UP_CF_ROUTER_RUNNING | START_UP_ESP_ROUTER_RUNNING | START_UP_WIFI_ROUTER_RUNNING);

  xTaskCreate(router_from_gap8, "Router from GAP8", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_crazyflie, "Router from CF", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_esp32, "Router from ESP32", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_wifi, "Router from WIFI", 10000, NULL, 1, NULL);

  ESP_LOGI("ROUTER", "Waiting for tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_GAP8_ROUTER_RUNNING |
                      START_UP_CF_ROUTER_RUNNING |
                      START_UP_ESP_ROUTER_RUNNING |
                      START_UP_WIFI_ROUTER_RUNNING,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  ESP_LOGI("ROUTER", "Initialized");
}
