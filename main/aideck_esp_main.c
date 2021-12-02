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
#include "driver/uart.h"
#include "esp_log.h"

#include "spi_transport.h"
#include "uart_transport.h"
#include "router.h"

/* The LED is connected on GPIO */
#define BLINK_GPIO 4
static uint32_t blink_period_ms = 500;

void test_echo(int count) {
    static spi_transport_packet_t packet;

    printf("Testing %d max-size pings ...\n", count);

    packet.length = SPI_TRANSPORT_MTU;
    packet.data[0] = 0x01;
    spi_transport_send(&packet);

    int start  = xTaskGetTickCount();
    for (int i=0; i<100; i++) {
        packet.length = 64;
        packet.data[0] = 0x01;
        spi_transport_send(&packet);
        spi_transport_receive(&packet);
    }

    spi_transport_receive(&packet);

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
    packet.data[2] = 62;

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
        packet.length = SPI_TRANSPORT_MTU;
        packet.data[0] = 0x00;
        spi_transport_send(&packet);
    }
    int stop = xTaskGetTickCount();

    int ticks = stop - start;
    float runtime = (float)(stop - start) / (float)xPortGetTickRateHz();
    float pk_per_seconds = count / runtime;
    printf("Done in %f ms, %f pk/s, %f B/s\n", runtime * 1000, pk_per_seconds, pk_per_seconds * SPI_TRANSPORT_MTU);
}

int my_vprintf(const char * fmt, va_list ap) {
    int len = vprintf("Hello: ", ap);
    len += vprintf(fmt, ap);
    return len;
}

/*void app_main(void)
{

    esp_log_set_vprintf(my_vprintf);

    printf("Hello world!\n");

    //Print chip information
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

    vTaskDelay(200);

    //test_sink(100);
     //test_echo(1);

    static spi_transport_packet_t packet;
    static spi_transport_packet_t packet_rx;

    packet.length = 6;
    packet.data[0] = 1;
    packet.data[1] = 'B';
    packet.data[2] = 'C';
    packet.data[3] = 'D';
    packet.data[4] = 'E';
    packet.data[5] = 'F';

    spi_transport_send(&packet);

    spi_transport_receive(&packet_rx);

    printf("RX len is %u\n", packet_rx.length);

    for (int i = 0; i < 6; i++) {
      printf("%c ", packet_rx.data[i]);
    }

    printf("\n");

    //test_source();

    while(1) {
        vTaskDelay(1000);
    }
    esp_restart();
}*/

#define DEBUG_TXD_PIN (GPIO_NUM_0) // Nina 27 /SYSBOOT) => 0
//#define DEBUG_RXD_PIN (GPIO_NUM_12) // Nina 36 => 12

// 27 => 0
// 34 => 35

int a = 1;

void app_main(void)
{




    esp_log_level_set("SPI", ESP_LOG_DEBUG);
    //esp_log_set_vprintf(my_vprintf);

    ESP_LOGW("SPI", "Initializing the SPI transport!\n");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    // Intalling GPIO ISR service so that other parts of the code can
    // setup individual GPIO interrupt routines
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    spi_transport_init();
    //test_echo(1);

        const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 1000, 1000, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 0, 25, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_transport_init();

    router_init();

    //test_uart();

    vTaskDelay(200);

    //test_sink(10000);
    // test_echo(100);
    // test_source();

    while(1) {
        vTaskDelay(10);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(10);
        gpio_set_level(BLINK_GPIO, 0);
        //ESP_LOGW("SPI", "Wooo\n");

        //uart_transport_send(&tp);
    }
    esp_restart();
}