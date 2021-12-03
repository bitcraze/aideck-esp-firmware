#pragma once

#include <stdint.h>
#include <stddef.h>

#define UART_TRANSPORT_MTU 100

typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t data[UART_TRANSPORT_MTU];
} __attribute__((packed)) uart_transport_packet_t;

typedef struct {
  uint8_t start;
  uint8_t length; // Length of data - 2
  uint8_t dst;
  uint8_t src;
  uint8_t data[UART_TRANSPORT_MTU - 2]
} __attribute__((packed)) uart_transport_routable_packet_t;

void uart_transport_init();

void uart_transport_send(const uart_transport_packet_t *packet);

void uart_transport_receive(uart_transport_packet_t *packet);
