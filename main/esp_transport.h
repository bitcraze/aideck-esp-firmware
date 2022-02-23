#pragma once

// The ESP transport module represents a virtual transport link between the router and application code executing
// in the ESP

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

#include "spi_transport.h"

#define ESP_TRANSPORT_MTU (SPI_TRANSPORT_MTU)

#if ESP_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "ESP MTU bigger than defined by CPX"
#endif

typedef struct {
    uint16_t length;
    uint8_t data[ESP_TRANSPORT_MTU];
} __attribute__((packed)) esp_packet_t;

typedef struct {
    uint16_t length;
    CPXRouting_t route;
    uint8_t data[ESP_TRANSPORT_MTU - CPX_ROUTING_INFO_SIZE];
} __attribute__((packed)) esp_routable_packet_t;


void espTransportInit();


// Interface used by ESP applications to exchange CPX packets with other part of the system
void espAppSendToRouterBlocking(const esp_routable_packet_t* packet);
void espAppReceiveFromRouter(esp_routable_packet_t* packet);


// Interface used by the router
void espTransportSend(const uint8_t* data, const uint16_t dataLen);
uint16_t espTransportReceive(uint8_t* data);
