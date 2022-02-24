#pragma once

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

// The SPI transport module represents the transport link between the router and the GAP8 module on the AI-deck.

#define SPI_TRANSPORT_MTU 1022

#if SPI_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "SPI MTU bigger than defined by CPX"
#endif

typedef struct {
    uint16_t dataLength;
    CPXRoutingPacked_t route;
    uint8_t data[SPI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) SpiStructuredData_t;

typedef union {
    uint8_t raw[SPI_TRANSPORT_MTU + 2];
    SpiStructuredData_t structuredData;
} __attribute__((packed)) SpiBuffer_t;

void spi_transport_init();

// Interface used by the router
void spi_transport_send(const CPXRoutablePacket_t* packet);
void spi_transport_receive(CPXRoutablePacket_t* packet);
