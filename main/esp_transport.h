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

// The ESP transport module represents a virtual transport link between the router and application code executing
// in the ESP

#include <stdint.h>
#include <stddef.h>
#include "cpx.h"

#include "spi_transport.h"

#define ESP_TRANSPORT_MTU (SPI_TRANSPORT_MTU)

#if ESP_TRANSPORT_MTU > CPX_MAX_PAYLOAD_SIZE
    #pragma warn "ESP MTU bigger than defined by CPX"
#endif

// esp_routable_packet_t is identical to CPXRoutablePacket_t
typedef CPXRoutablePacket_t esp_routable_packet_t;

void espTransportInit();


// Interface used by ESP applications to exchange CPX packets with other part of the system
void espAppSendToRouterBlocking(const esp_routable_packet_t* packet);
void espAppReceiveFromRouter(esp_routable_packet_t* packet);


// Interface used by the router
void espTransportSend(const CPXRoutablePacket_t* packet);
void espTransportReceive(CPXRoutablePacket_t* packet);
