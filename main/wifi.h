#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

#define WIFI_TRANSPORT_MTU 1022

#if WIFI_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "WIFI MTU bigger than defined by CPX"
#endif

typedef struct {
    uint16_t length;
    uint8_t data[WIFI_TRANSPORT_MTU];
} __attribute__((packed)) wifi_transport_packet_t;


void wifi_init();

void wifi_transport_send(const uint8_t* data, const uint16_t dataLen);

uint16_t wifi_transport_receive(uint8_t* data);
