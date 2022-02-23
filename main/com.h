#pragma once

#include <stdint.h>
#include <stddef.h>

#include "cpx.h"
#include "esp_transport.h"


void com_init();

void com_receive_test_blocking(esp_routable_packet_t * packet);

void com_receive_wifi_ctrl_blocking(esp_routable_packet_t * packet);
