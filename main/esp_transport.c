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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "esp_transport.h"


// This is probably too big, but let's keep things simple....
#define ESP_ROUTER_TX_QUEUE_LENGTH (4)
#define ESP_ROUTER_RX_QUEUE_LENGTH (4)

static xQueueHandle espRxQueue;
static xQueueHandle espTxQueue;


void espTransportInit() {
  espRxQueue = xQueueCreate(ESP_ROUTER_RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
  espTxQueue = xQueueCreate(ESP_ROUTER_TX_QUEUE_LENGTH, sizeof(esp_routable_packet_t));

  ESP_LOGI("ESP_TRNSP", "Initialized");
}

void espAppSendToRouterBlocking(const esp_routable_packet_t* packet) {
  xQueueSend(espTxQueue, packet, portMAX_DELAY);
}

void espAppReceiveFromRouter(esp_routable_packet_t* packet) {
  CPXRoutablePacket_t* buf = packet;
  xQueueReceive(espRxQueue, buf, portMAX_DELAY);
}

void espTransportSend(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= ESP_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(espRxQueue, packet, portMAX_DELAY);
}

void espTransportReceive(CPXRoutablePacket_t* packet) {
  esp_routable_packet_t* buf = packet;
  xQueueReceive(espTxQueue, buf, portMAX_DELAY);
  packet->route.lastPacket = true;
}
