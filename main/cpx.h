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

// This enum is used to identify source and destination for CPX routing information
typedef enum {
  STM32 = 1, // The STM in the Crazyflie
  ESP32 = 2, // The ESP on the AI-deck
  HOST = 3,  // A remote computer connected via Wifi
  GAP8 = 4   // The GAP8 on the AI-deck
} __attribute__((packed)) CPXTarget_t;

typedef enum {
  SYSTEM = 1,
  CONSOLE = 2,
  CRTP = 3,
  WIFI_CTRL = 4,
  APP = 5,
  TEST = 0x0E,
  BOOTLOADER = 0x0F,
} __attribute__((packed)) CPXFunction_t;

typedef struct {
  uint8_t destination;
  uint8_t source;
  uint8_t function;
} CPXRouting_t;

// This struct contains routing information in a packed format. This struct
// should mainly be used to serialize data when tranfering. Unpacked formats
// should be preferred in application code.
typedef struct {
  uint8_t destination : 4;
  uint8_t source : 4;
  uint8_t function;
} __attribute__((packed)) CPXRoutingPacked_t;

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

// The maximum MTU of any link
#define CPX_MAX_PAYLOAD_SIZE 1022


typedef struct {
  CPXRouting_t route;

  uint16_t dataLength;
  uint8_t data[CPX_MAX_PAYLOAD_SIZE - CPX_ROUTING_PACKED_SIZE];
} CPXRoutablePacket_t;
