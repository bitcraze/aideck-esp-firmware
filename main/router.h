#pragma once

#include <stdint.h>
#include <stddef.h>

#include "com.h"

#define CF_TARGET   (0x0)
#define GAP8_TARGET (0x2)
#define ESP32_TARGET  (0x1)

void router_init();
