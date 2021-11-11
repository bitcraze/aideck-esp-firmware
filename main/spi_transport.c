// SPI Transport implementation

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#define GAP_RTT_GPIO 32
#define ESP_RTT_GPIO 2

#define SPI_CS_GPIO 5
#define SPI_MOSI_GPIO 19
#define SPI_MISO_GPIO 23
#define SPI_SCLK_GPIO 18

#define SPI_BUFFER_LEN 68

static char * tx_buffer;
static char * rx_buffer;

static xQueueHandle tx_queue;
static xQueueHandle rx_queue;

xSemaphoreHandle gap_rtt_event;
static TaskHandle_t spi_task_handle;

void IRAM_ATTR gap_rtt_enabled_handler(void * _param) {
    int task_woken = 0;

    xSemaphoreGiveFromISR(gap_rtt_event, &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

static IRAM_ATTR void spi_post_setup(struct spi_slave_transaction_t * _transaction) {
    gpio_set_level(ESP_RTT_GPIO, 1);
}

static IRAM_ATTR void spi_post_transfer(struct spi_slave_transaction_t * _transaction) {
    gpio_set_level(ESP_RTT_GPIO, 0);
}

static void spi_task(void* _param) {
    while(1) {
        xSemaphoreTake(gap_rtt_event, portMAX_DELAY);

        tx_buffer[0] = 20;
        for (int i=0; i<20; i++) {
            tx_buffer[i+1] = i+20;
            rx_buffer[i+1] = i+40;
        }

        spi_slave_transaction_t transaction = {
            .length = SPI_BUFFER_LEN*8,
            .tx_buffer = tx_buffer,
            .rx_buffer = rx_buffer,
        };

        printf("Transaction ...\n");
        spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);

        printf("Transferred: %d\n", transaction.trans_len);

        int len  = rx_buffer[0];
        printf("Rx[%d]", rx_buffer[0]);
        for (int i=0; i<len; i++) {
            printf(" %02x", rx_buffer[i+1]);
        }
        printf(" | ");
        for (int i=len; i<len+10; i++) {
            printf(" %02x", rx_buffer[i+1]);
        }
        printf("\n");
    }
}

void spi_init() {
    printf("Initializing the SPI transport!\n");

    // Setting up synchronization routines
    gap_rtt_event = xSemaphoreCreateBinary();

    // Setting up GPIOs
    gpio_config_t gap_rtt_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull<<GAP_RTT_GPIO)
    };
    gpio_config(&gap_rtt_conf);
    gpio_isr_handler_add(GAP_RTT_GPIO, gap_rtt_enabled_handler, NULL);
    gpio_intr_enable(GAP_RTT_GPIO);

    gpio_config_t esp_rtt_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ull<<ESP_RTT_GPIO),
    };
    gpio_config(&esp_rtt_conf);
    gpio_set_level(ESP_RTT_GPIO, 0);

    // Setting up SPI
    spi_bus_config_t spi_config = {
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .sclk_io_num = SPI_SCLK_GPIO,
    };
    
    spi_slave_interface_config_t spi_slave_config = {
        .mode = 0,
        .spics_io_num = SPI_CS_GPIO,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = spi_post_setup,
        .post_trans_cb = spi_post_transfer,
    };

    spi_slave_initialize(VSPI_HOST, &spi_config, &spi_slave_config, 1);

    // Allocating buffers
    tx_buffer = (char*)heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);
    rx_buffer = (char*)heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);

    // Launching SPI communication task
    xTaskCreate(spi_task, "SPI transport", 5000, NULL, 1, &spi_task_handle);    


    // vTaskDelay(100);

    // xSemaphoreGive(gap_rtt_event);

    // while (1) {
    //     int value = gpio_get_level(GAP_RTT_GPIO);
    //     printf("Pin value: %d\n", value);

    //     vTaskDelay(100);
    // }
}

void spi_transport_send(char* packet, int length) {
    // Todo
}

void spi_transport_receive(char* packet, int length) {
    // Todo
}