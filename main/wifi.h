/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

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
