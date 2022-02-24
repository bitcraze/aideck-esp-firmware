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

// esp_routable_packet_t is identical to CPXRoutablePacket_t
typedef CPXRoutablePacket_t esp_routable_packet_t;

void espTransportInit();


// Interface used by ESP applications to exchange CPX packets with other part of the system
void espAppSendToRouterBlocking(const esp_routable_packet_t* packet);
void espAppReceiveFromRouter(esp_routable_packet_t* packet);


// Interface used by the router
void espTransportSend(const CPXRoutablePacket_t* packet);
void espTransportReceive(CPXRoutablePacket_t* packet);
