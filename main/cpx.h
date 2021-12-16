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

/*typedef struct {
    CPXRouting_t routing;
    uint8_t data[MTU];
} __attribute__((packed)) CPXPacket_t;*/

