#pragma once

// The UART transport module represents the transport link between the router and the STM on the Crazyflie.

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

#define UART_TRANSPORT_MTU 100

#if UART_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "UART MTU bigger than defined by CPX"
#endif


typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength;
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[UART_TRANSPORT_MTU];
    };
} __attribute__((packed)) uart_transport_packet_t;


void uart_transport_init();

// Interface used by the router
void uart_transport_send(const CPXRoutablePacket_t* packet);
void uart_transport_receive(CPXRoutablePacket_t* packet);
