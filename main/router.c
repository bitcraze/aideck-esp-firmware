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
static uint8_t wifiRxBuf[WIFI_TRANSPORT_MTU];

static RouteContext_t gap8_task_context;
static uint8_t spiRxBuf[SPI_TRANSPORT_MTU];

static RouteContext_t cf_task_context;
static uint8_t uartRxBuf[UART_TRANSPORT_MTU];

static RouteContext_t esp_task_context;
static uint8_t espRxBuf[ESP_TRANSPORT_MTU];

typedef uint16_t (*Receiver_t)(uint8_t* data);
typedef void (*Sender_t)(const uint8_t* data, const uint16_t dataLength);

static const int START_UP_GAP8_ROUTER_RUNNING = BIT0;
static const int START_UP_CF_ROUTER_RUNNING = BIT1;
static const int START_UP_ESP_ROUTER_RUNNING = BIT2;
static const int START_UP_WIFI_ROUTER_RUNNING = BIT3;
static EventGroupHandle_t startUpEventGroup;

static void splitAndSend(const CPXRoutablePacket_t* rxp, const uint16_t cpxDataLength, RouteContext_t* context, Sender_t sender) {
  // TODO krri Split packet if needed
  context->txp.route = rxp->route;
  memcpy(context->txp.data, rxp->data, cpxDataLength);

  const uint16_t transportBufLength = cpxDataLength + CPX_ROUTING_INFO_SIZE;
  sender((uint8_t*)&context->txp, transportBufLength);
}

static void route(Receiver_t receive, uint8_t* rxBuf, RouteContext_t* context) {
  while(1) {
    uint16_t rxBufLength = receive(rxBuf);

    CPXRoutablePacket_t* rxp = (CPXRoutablePacket_t*)rxBuf;
    const uint16_t cpxDataLength = rxBufLength - CPX_ROUTING_INFO_SIZE;

    switch (rxp->route.destination) {
      case GAP8:
        ESP_LOGD("ROUTER", "[0x%02X] -> GAP8 [0x%02X] (%u)", rxp->route.source, rxp->route.destination, cpxDataLength);
        splitAndSend(rxp, rxBufLength, context, spi_transport_send);
        break;
      case STM32:
        ESP_LOGD("ROUTER", "[0x%02X] -> STM32 [0x%02X] (%u)", rxp->route.source, rxp->route.destination, cpxDataLength);
        splitAndSend(rxp, rxBufLength, context, uart_transport_send);
        break;
      case ESP32:
        ESP_LOGD("ROUTER", "[0x%02X] -> ESP32 [0x%02X] (%u)", rxp->route.source, rxp->route.destination, cpxDataLength);
        splitAndSend(rxp, rxBufLength, context, espTransportSend);
        break;
      case HOST:
        ESP_LOGD("ROUTER", "[0x%02X] -> HOST [0x%02X] (%u)", rxp->route.source, rxp->route.destination, cpxDataLength);
        splitAndSend(rxp, rxBufLength, context, wifi_transport_send);
        break;
      default:
        ESP_LOGW("ROUTER", "Cannot route from [0x%02X] to [0x%02X]", rxp->route.source, rxp->route.destination);
    }
  }
}

static void router_from_gap8(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_GAP8_ROUTER_RUNNING);
  route(spi_transport_receive, spiRxBuf, &gap8_task_context);
}

static void router_from_crazyflie(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_CF_ROUTER_RUNNING);
  route(uart_transport_receive, uartRxBuf, &cf_task_context);
}

static void router_from_esp32(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_ESP_ROUTER_RUNNING);
  route(espTransportReceive, espRxBuf, &esp_task_context);
}

static void router_from_wifi(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_WIFI_ROUTER_RUNNING);
  // TODO: krri Fix this (wifi needs to be split, since it's being used from routing and using routing)
  vTaskDelay(100);

  route(wifi_transport_receive, wifiRxBuf, &wifi_task_context);
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
