#pragma once

#include <stdint.h>
#include <stddef.h>

typedef enum {
    CF_CONSOLE = 0x01,
    CF_APP = 0x02,
    AIDECK_ESP_WIFI_DATA = 0x11,
    AIDECK_ESP_WIFI_CTRL = 0x12,
    AIDECK_ESP_PM = 0x13,
    AIDECK_ESP_APP = 0x14,
    AIDECK_ESP_TEST = 0x15,
    AIDECK_GAP8_APP = 0x21,
    AIDECK_GAP8_TEST = 0x22,
    AIDECK_GAP8_BOOTLOADER = 0x23
} __attribute__((packed)) AIDeckTransmitIdentity_t;

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
    AIDeckTransmitIdentity_t dst;
    AIDeckTransmitIdentity_t src;
    uint8_t data[ESP_ROUTABLE_PACKET_SIZE-2];
} __attribute__((packed)) esp_routable_packet_t;

void com_init();

void com_receive_test_blocking(esp_routable_packet_t * packet);

void com_receive_wifi_ctrl_blocking(esp_routable_packet_t * packet);

void com_receive_wifi_data_blocking(esp_routable_packet_t * packet);

void com_send_blocking(esp_routable_packet_t * packet);

void com_router_post_packet(esp_packet_t * packet);

void com_router_get_packet(esp_packet_t * packet);