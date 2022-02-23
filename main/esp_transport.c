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
#define ESP_ROUTER_QUEUE_SIZE (sizeof(esp_routable_packet_t))

static xQueueHandle espRxQueue;
static xQueueHandle espTxQueue;

static xSemaphoreHandle transportSendLock;
static StaticSemaphore_t transportSendLockBuffer;


void espTransportInit() {
  espRxQueue = xQueueCreate(ESP_ROUTER_RX_QUEUE_LENGTH, ESP_ROUTER_QUEUE_SIZE);
  espTxQueue = xQueueCreate(ESP_ROUTER_TX_QUEUE_LENGTH, ESP_ROUTER_QUEUE_SIZE);

  transportSendLock = xSemaphoreCreateMutexStatic(&transportSendLockBuffer);
  configASSERT(transportSendLock);

  ESP_LOGI("ESP_TRNSP", "Initialized");
}

void espAppSendToRouterBlocking(const esp_routable_packet_t* packet) {
    xQueueSend(espTxQueue, packet, portMAX_DELAY);
}

void espAppReceiveFromRouter(esp_routable_packet_t* packet) {
    xQueueReceive(espRxQueue, packet, portMAX_DELAY);
}

void espTransportSend(const uint8_t* data, const uint16_t dataLen) {
  static esp_routable_packet_t txBuffer;

  assert(dataLen <= ESP_TRANSPORT_MTU);

  xSemaphoreTake(transportSendLock, portMAX_DELAY);

  CPXRoutablePacket_t* rxp = (CPXRoutablePacket_t*)data;

  txBuffer.length = dataLen - CPX_ROUTING_INFO_SIZE;
  // TOOD krri unpack? Both esp_routable_packet_t and CPXRoutablePacket_t are packed so this is fine for now. esp_routable_packet_t should be changed to not packed though.
  txBuffer.route = rxp->route;
  memcpy(txBuffer.data, rxp->data, txBuffer.length);
  xSemaphoreGive(transportSendLock);

  xQueueSend(espRxQueue, &txBuffer, portMAX_DELAY);
}

uint16_t espTransportReceive(uint8_t* data) {
  static esp_routable_packet_t rxBuffer;

  xQueueReceive(espTxQueue, &rxBuffer, portMAX_DELAY);
  // TODO krri pack
  CPXRoutablePacket_t* txp = (CPXRoutablePacket_t*)data;

  txp->route = rxBuffer.route;
  memcpy(txp->data, rxBuffer.data, rxBuffer.length);

  return rxBuffer.length + CPX_ROUTING_INFO_SIZE;
}
