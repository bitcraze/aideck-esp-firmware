#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

#define WIFI_TRANSPORT_MTU 1022

typedef struct {
    uint16_t length;
    uint8_t data[WIFI_TRANSPORT_MTU];
} __attribute__((packed)) wifi_transport_packet_t;

typedef struct {
    uint16_t length;
    CPXRouting_t route;
    uint8_t data[WIFI_TRANSPORT_MTU-2];
} __attribute__((packed)) wifi_transport_routable_packet_t;

void wifi_init();

void wifi_transport_send(const wifi_transport_packet_t *packet);

void wifi_transport_receive(wifi_transport_packet_t *packet);