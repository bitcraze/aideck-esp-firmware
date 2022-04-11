#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "mdns.h"

void discovery_init()
{
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGW("DISCOVERY", "MDNS Init failed: %d\n", err);
        return;
    }

    uint8_t mac[6];
    char   *hostname;
    char *macString;
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (-1 == asprintf(&hostname, "aideck-%02X%02X%02X", mac[3], mac[4], mac[5])) {
        abort();
    }
    if (-1 == asprintf(&macString, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5])) {
        abort();
    }
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