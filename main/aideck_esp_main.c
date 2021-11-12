/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include "spi_transport.h"

void test_echo(int count) {
    static spi_transport_packet_t packet;

    printf("Testing %d max-size pings ...\n", count);

    int start  = xTaskGetTickCount();
    for (int i=0; i<100; i++) {
        packet.length = 64;
        packet.data[0] = 0x01;
        spi_transport_send(&packet);
        spi_transport_receive(&packet);
    }
    int stop = xTaskGetTickCount();

    int ticks = stop - start;
    float runtime = (float)(stop - start) / (float)xPortGetTickRateHz();
    float ping_per_seconds = count / runtime;
    printf("Done in %f ms, %f ping/s\n", runtime * 1000, ping_per_seconds);
}

void test_source() {
    static spi_transport_packet_t packet;

    printf("Testing sourcing 100 packets ...\n");

    packet.length = 10;
    packet.data[0] = 0x02;
    packet.data[1] = 100;
    packet.data[2] = 10;

    spi_transport_send(&packet);

    for (int i=0; i<100; i++) {
        spi_transport_receive(&packet);
    }
    printf("Done!\n");
}

void test_sink(int count) {
    static spi_transport_packet_t packet;
    
    printf("Testing %d packet TX\n", count);

    int start  = xTaskGetTickCount();
    for (int i=0; i<count; i++) {
        packet.length = 64;
        packet.data[0] = 0x00;
        spi_transport_send(&packet);
    }
    int stop = xTaskGetTickCount();

    int ticks = stop - start;
    float runtime = (float)(stop - start) / (float)xPortGetTickRateHz();
    float pk_per_seconds = count / runtime;
    printf("Done in %f ms, %f pk/s, %f B/s\n", runtime * 1000, pk_per_seconds, pk_per_seconds * 64);
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    // Intalling GPIO ISR service so that other parts of the code can
    // setup individual GPIO interrupt routines
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    spi_transport_init();

    vTaskDelay(10);

    test_sink(1000);

    while(1) {
        vTaskDelay(1000);
    }
    esp_restart();
}
