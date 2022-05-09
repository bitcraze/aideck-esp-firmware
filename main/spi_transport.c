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

#define SPI_BUFFER_LEN (sizeof(SpiBuffer_t) + 2)

#define TX_QUEUE_LENGTH 10
#define RX_QUEUE_LENGTH 10

//#define DEBUG(...) ESP_LOGD("SPI", __VA_ARGS__)
#define DEBUG(...) 

static SpiBuffer_t* tx_buffer;
static SpiBuffer_t* rx_buffer;

static xQueueHandle tx_queue;
static xQueueHandle rx_queue;

#define TASK_EVENT (1<<0)
static EventGroupHandle_t task_event;

static const int START_UP_MAIN_TASK = BIT0;
static EventGroupHandle_t startUpEventGroup;

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

static IRAM_ATTR void spi_post_transfer(struct spi_slave_transaction_t * _transaction) {
    gpio_set_level(ESP_RTT_GPIO, 0);
}

static void spi_task(void* _param) {
    static CPXRoutablePacket_t qPacket;

    xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);
    while(1) {
        if (uxQueueMessagesWaiting(tx_queue) == 0) {
            DEBUG("Waiting for events ...");
            xEventGroupWaitBits(task_event, TASK_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
        }

        // Check if we can send a packet
        if (xQueueReceive(tx_queue, &qPacket, 0) == pdTRUE) {
            DEBUG("Some data to send ...");
            // Set the length byte and copy the packet to the TX buffer
            const uint16_t payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
            tx_buffer->structuredData.dataLength = payloadLength;

            cpxRouteToPacked(&qPacket.route, &tx_buffer->structuredData.route);
            memcpy(tx_buffer->structuredData.data, qPacket.data, qPacket.dataLength);
        } else {
            // Nothing to send, length byte=0
            tx_buffer->structuredData.dataLength = 0;
        }

        rx_buffer->structuredData.dataLength = 0;

        DEBUG("About to send %d bytes", tx_buffer->raw[0] | (tx_buffer->raw[1] << 8));

        // Trigger the transfer!
        spi_slave_transaction_t transaction = {
            .length = SPI_BUFFER_LEN*8,
            .tx_buffer = tx_buffer->raw,
            .rx_buffer = rx_buffer->raw,
        };

        DEBUG("Transaction set up");
        spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);

        DEBUG("Transaction done: %dB", transaction.trans_len/8);

        int rx_len  = rx_buffer->structuredData.dataLength;

        // If there is some data received, push the packet in the RX queue!
        if (rx_len != 0) {
            qPacket.dataLength = rx_len - CPX_ROUTING_PACKED_SIZE;

            cpxPackedToRoute(&rx_buffer->structuredData.route, &qPacket.route);

            memcpy(qPacket.data, &rx_buffer->structuredData.data, qPacket.dataLength);
            xQueueSend(rx_queue, &qPacket, portMAX_DELAY);
        }
    }
}

void spi_transport_init() {

    // Setting up synchronization items
    tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

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
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull<<SPI_CS_GPIO)
    };
    gpio_config(&spi_cs_int_config);

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
    // TODO krri Malloc?
    tx_buffer = heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);
    rx_buffer = heap_caps_malloc(SPI_BUFFER_LEN, MALLOC_CAP_DMA);

    // Launching SPI communication task
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK);
    xTaskCreate(spi_task, "SPI transport", 5000, NULL, 1, &spi_task_handle);
    ESP_LOGI("SPI", "Waiting for task to start");
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_MAIN_TASK,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI("SPI", "Transport initialized");
}


void spi_transport_send(const CPXRoutablePacket_t* packet) {
    assert(packet->dataLength <= SPI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(tx_queue, packet, portMAX_DELAY);
    xEventGroupSetBits(task_event, TASK_EVENT);
}

void spi_transport_receive(CPXRoutablePacket_t* packet) {
    xQueueReceive(rx_queue, packet, portMAX_DELAY);
}
