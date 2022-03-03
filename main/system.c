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

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"

#include "system.h"

#include "com.h"

// NINA GPIO18 => ESP pin 27
#define GAP8_RST_GPIO 27

static void resetGAP8() {
  gpio_pad_select_gpio(GAP8_RST_GPIO);
  gpio_set_direction(GAP8_RST_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(GAP8_RST_GPIO, 0);
  vTaskDelay(10);
  gpio_reset_pin(GAP8_RST_GPIO);
}

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

static void system_task(void *pvParameters) {

  // Send jump to GAP8 firmware at startup
  cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_BOOTLOADER, &txp.route);
  txp.data[0] = 0x06;
  txp.dataLength = 1;
  espAppSendToRouterBlocking(&txp);

  while (1) {
    com_receive_system_blocking(&rxp);
    SystemCommandPacket_t * systemCommandPacket = (SystemCommandPacket_t*) rxp.data;

    switch (systemCommandPacket->cmd) {
      case SYSTEM_RESET_GAP8:
        resetGAP8();
        cpxInitRoute(rxp.route.destination, rxp.route.source, CPX_F_SYSTEM, &txp.route);
        txp.data[0] = SYSTEM_RESET_GAP8;
        txp.dataLength = 1;
        espAppSendToRouterBlocking(&txp);
        break;
      default:
        printf("Not handing system command 0x%X\n", systemCommandPacket->cmd);
        break;
    }
  }
}

void system_init() {
  xTaskCreate(system_task, "System task", 5000, NULL, 1, NULL);

  ESP_LOGI("SYS", "Initialized");
}