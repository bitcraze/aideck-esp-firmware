#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

// This is probably too much, but it makes things a lot simpler
// and we have the memory (for now...)
#include "spi_transport.h"
#define ESP_PACKET_SIZE (SPI_TRANSPORT_MTU)

#if ESP_PACKET_SIZE > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "ESP MTU bigger than defined by CPX"
#endif

typedef struct {
    uint16_t length;
    uint8_t data[ESP_PACKET_SIZE];
} __attribute__((packed)) esp_packet_t;

typedef struct {
    uint16_t length;
    CPXRouting_t route;
    uint8_t data[ESP_PACKET_SIZE - CPX_ROUTING_INFO_SIZE];
} __attribute__((packed)) esp_routable_packet_t;

void com_init();

void com_receive_test_blocking(esp_routable_packet_t * packet);

void com_receive_wifi_ctrl_blocking(esp_routable_packet_t * packet);

void com_send_blocking(esp_routable_packet_t * packet);

void com_router_post_packet(const uint8_t* data, const uint16_t dataLen);

uint16_t com_router_get_packet(uint8_t* data);
