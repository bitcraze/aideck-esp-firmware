// SPI Transport implementation

#include "spi_transport.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_log.h"

#define GAP_RTT_GPIO 32
#define ESP_RTT_GPIO 2

#define SPI_CS_GPIO 5
#define SPI_MOSI_GPIO 19
#define SPI_MISO_GPIO 23
#define SPI_SCLK_GPIO 18

#define SPI_BUFFER_LEN (SPI_TRANSPORT_MTU + 2)

#define TX_QUEUE_LENGTH 10
#define RX_QUEUE_LENGTH 10

//#define DEBUG(...) ESP_LOGI("SPI", __VA_ARGS__)
#define DEBUG(...)

static char * tx_buffer;
static char * rx_buffer;

static xQueueHandle tx_queue;
static xQueueHandle rx_queue;

static EventGroupHandle_t task_event;

#define TASK_EVENT (1<<0)

static TaskHandle_t spi_task_handle;

void IRAM_ATTR gap_rtt_enabled_handler(void * _param) {
    int task_woken = 0;

    xEventGroupSetBitsFromISR(task_event, TASK_EVENT, &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

volatile int plop;
static IRAM_ATTR void spi_post_setup(struct spi_slave_transaction_t * _transaction) {
    gpio_set_level(ESP_RTT_GPIO, 1);
}

static IRAM_ATTR void  spi_transaction_started_handler(void * _param) {
    gpio_set_level(ESP_RTT_GPIO, 0);
}

static IRAM_ATTR void spi_post_transfer(struct spi_slave_transaction_t * _transaction) {
    gpio_set_level(ESP_RTT_GPIO, 0);
}

static void spi_task(void* _param) {
    static spi_transport_packet_t packet;

    while(1) {
        if (uxQueueMessagesWaiting(tx_queue) == 0) {
            DEBUG("Waiting for events ...\n");
            int bits = xEventGroupWaitBits(task_event, TASK_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
            // xEventGroupClearBits(task_event, TASK_EVENT);
            DEBUG("Event! %02x\n", bits);
        }

        // Check if we can send a packet
        if (xQueueReceive(tx_queue, &packet, 0) == pdTRUE) {
            DEBUG("Some data to send ...\n");
            // Set the length byte and copy the packet to the TX buffer
            tx_buffer[0] = packet.length;
            tx_buffer[1] = packet.length >> 8;
            memcpy(&tx_buffer[2], packet.data, packet.length);
        } else {
            // Nothing to send, length byte=0
            tx_buffer[0] = 0;
            tx_buffer[1] = 0;
        }

        DEBUG("About to send %d bytes\n", tx_buffer[0] | (tx_buffer[1] << 8));

        // Trigger the transfer!
        spi_slave_transaction_t transaction = {
            .length = SPI_BUFFER_LEN*8,
            .tx_buffer = tx_buffer,
            .rx_buffer = rx_buffer,
        };

        DEBUG("Transaction ...\n");
        spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);

        DEBUG("Transferred: %dB\n", transaction.trans_len/8);

        int rx_len  = rx_buffer[0] + (((int)rx_buffer[1]) << 8);
        DEBUG("Rx[%d]\n", rx_buffer[0]);
        for (int i=0; i<rx_len; i++) {
            DEBUG(" %02x", rx_buffer[i+1]);
        }
        DEBUG("\n");

        // If there is some data received, push the packet in the RX queue!
        if (rx_len != 0) {
            packet.length = rx_len;
            memcpy(packet.data, &rx_buffer[2], packet.length);
            xQueueSend(rx_queue, &packet, portMAX_DELAY);
        }
    }
}

void spi_transport_init() {

    // Setting up synchronization items
    tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(spi_transport_packet_t));
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(spi_transport_packet_t));

    task_event = xEventGroupCreate();

    // Setting up GPIOs
    gpio_config_t gap_rtt_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull<<GAP_RTT_GPIO)
    };
    gpio_config(&gap_rtt_conf);
    gpio_isr_handler_add(GAP_RTT_GPIO, gap_rtt_enabled_handler, NULL);
    gpio_intr_enable(GAP_RTT_GPIO);

    gpio_config_t spi_cs_int_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull<<SPI_CS_GPIO)
    };
    gpio_config(&spi_cs_int_config);
    gpio_isr_handler_add(SPI_CS_GPIO, spi_transaction_started_handler, NULL);
    gpio_intr_enable(SPI_CS_GPIO);

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
    xTaskCreate(spi_task, "SPI transport", 10000, NULL, 1, &spi_task_handle);
    ESP_LOGI("SPI", "Transport initialized");
}

void spi_transport_send(const spi_transport_packet_t *packet) {
    xQueueSend(tx_queue, packet, portMAX_DELAY);
    xEventGroupSetBits(task_event, TASK_EVENT);
}

void spi_transport_receive(spi_transport_packet_t *packet) {
    xQueueReceive(rx_queue, packet, portMAX_DELAY);
}
