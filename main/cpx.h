#pragma once

#include <stdint.h>

typedef enum {
  STM32 = 1,
  ESP32 = 2,
  HOST = 3,
  GAP8 = 4
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
  uint8_t destination : 4;
  uint8_t source : 4;
  uint8_t function;
} __attribute__((packed)) CPXRouting_t;

#define CPX_ROUTING_INFO_SIZE (sizeof(CPXRouting_t))

// The maximum MTU of any link
#define CPX_MAX_PAYLOAD_SIZE 1022


typedef struct {
    CPXRouting_t route;
    uint8_t data[CPX_MAX_PAYLOAD_SIZE - CPX_ROUTING_INFO_SIZE];
} __attribute__((packed)) CPXRoutablePacket_t;
