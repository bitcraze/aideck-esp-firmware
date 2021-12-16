#pragma once

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"


#define SPI_TRANSPORT_MTU 1022

typedef struct {
    size_t length;
    uint8_t data[SPI_TRANSPORT_MTU];
} __attribute__((packed)) spi_transport_packet_t;

typedef struct {
    size_t length;
    CPXRouting_t route;
    uint8_t data[SPI_TRANSPORT_MTU-2];
} __attribute__((packed)) spi_transport_routable_packet_t;

void spi_transport_init();

void spi_transport_send(const spi_transport_packet_t *packet);

void spi_transport_receive(spi_transport_packet_t *packet);
