#pragma once

#include <stdint.h>
#include <stddef.h>

#define SPI_TRANSPORT_MTU 255

typedef struct {
    size_t length;
    uint8_t data[SPI_TRANSPORT_MTU];
} spi_transport_packet_t;

void spi_transport_init();

void spi_transport_send(const spi_transport_packet_t *packet);

void spi_transport_receive(spi_transport_packet_t *packet);
