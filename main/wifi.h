#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"

#define WIFI_TRANSPORT_MTU 1022

#if WIFI_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "WIFI MTU bigger than defined by CPX"
#endif

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) WifiTransportPayload_t;

typedef struct {
    uint16_t payloadLength;
    union {
        WifiTransportPayload_t routablePayload;
        uint8_t payload[WIFI_TRANSPORT_MTU];
    };
} __attribute__((packed)) WifiTransportPacket_t;


void wifi_init();

// Interface used by the router
void wifi_transport_send(const CPXRoutablePacket_t* packet);
void wifi_transport_receive(CPXRoutablePacket_t* packet);
