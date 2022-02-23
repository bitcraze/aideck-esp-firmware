#pragma once

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

#define UART_TRANSPORT_MTU 100

#if UART_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "UART MTU bigger than defined by CPX"
#endif

typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t data[UART_TRANSPORT_MTU];
} __attribute__((packed)) uart_transport_packet_t;


void uart_transport_init();

void uart_transport_send(const uint8_t* data, const uint16_t dataLen);

uint16_t uart_transport_receive(uint8_t* data);
