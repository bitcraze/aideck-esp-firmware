#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

// This is probably too much, but it makes things a lot simpler
// and we have the memory (for now...)
#include "spi_transport.h"
#define ESP_ROUTABLE_PACKET_SIZE (SPI_TRANSPORT_MTU)

typedef struct {
    uint16_t length;
    uint8_t data[ESP_ROUTABLE_PACKET_SIZE];
} __attribute__((packed)) esp_packet_t;

typedef struct {
    uint16_t length;
    CPXRouting_t route;
    uint8_t data[ESP_ROUTABLE_PACKET_SIZE-2];
} __attribute__((packed)) esp_routable_packet_t;

void com_init();

void com_receive_test_blocking(esp_routable_packet_t * packet);

void com_receive_wifi_ctrl_blocking(esp_routable_packet_t * packet);

void com_send_blocking(esp_routable_packet_t * packet);

void com_router_post_packet(esp_packet_t * packet);

void com_router_get_packet(esp_packet_t * packet);