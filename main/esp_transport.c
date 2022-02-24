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
}
