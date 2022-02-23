#pragma once

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"


#define SPI_TRANSPORT_MTU 1022

#if SPI_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "SPI MTU bigger than defined by CPX"
#endif

typedef struct {
    size_t length;
    uint8_t data[SPI_TRANSPORT_MTU];
} __attribute__((packed)) spi_transport_packet_t;

void spi_transport_init();

void spi_transport_send(const uint8_t* data, const uint16_t dataLen);

uint16_t spi_transport_receive(uint8_t* data);
