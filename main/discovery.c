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
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "mdns.h"

static char hostname[14];
static char macString[13];

void discovery_init()
{
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGW("DISCOVERY", "MDNS Init failed: %d\n", err);
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    snprintf(hostname, sizeof(hostname), "aideck-%02X%02X%02X", mac[3], mac[4], mac[5]);
    snprintf(macString, sizeof(macString), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI("DISCOVERY", "Hostname is %s", hostname);
    mdns_hostname_set(hostname);
    mdns_instance_name_set("AI-deck");
    mdns_service_add(NULL, "_cpx", "_tcp", 5000, NULL, 0);
    mdns_service_instance_name_set("_cpx", "_tcp", "Crazyflie Packet eXchange");

    mdns_txt_item_t serviceTxtData[5] = {
        {"pname", "aideck"},
        {"pversion","1.1"},
        {"id", macString},
        {"name", ""},
        {"cpxversion", "1"}
    };
    mdns_service_txt_set("_cpx", "_tcp", serviceTxtData, 5);
}