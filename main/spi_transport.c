// SPI Transport implementation

#include "spi_transport.h"

#include <stdio.h>
#include <string.h>

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

#define SPI_BUFFER_LEN (SPI_TRANSPORT_MTU + 1)

#define TX_QUEUE_LENGTH 10
#define RX_QUEUE_LENGTH 10

static char * tx_buffer;
static char * rx_buffer;

static xSemaphoreHandle gap_rtt_event;
static xQueueHandle tx_queue;
static xQueueHandle rx_queue;
static xQueueSetHandle transport_queue_set;
static xSemaphoreHandle transfer_done;

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

    int task_woken = 0;

    xSemaphoreGiveFromISR(transfer_done, &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

static void spi_task(void* _param) {
    static spi_transport_packet_t packet;

    while(1) {
        // Wait for either gap to request a transfer or a TX packet to be ready
        xQueueSelectFromSet(transport_queue_set, portMAX_DELAY);

        // if we where unlocked by gap_rtt, make sure to take the semaphore so that it can be given again
        xSemaphoreTake(gap_rtt_event, 0);

        // Check if we can send a packet
        if (xQueueReceive(tx_queue, &packet, 0) == pdTRUE) {
            // Set the length byte and copy the packet to the TX buffer
            tx_buffer[0] = packet.length;
            memcpy(&tx_buffer[1], packet.data, packet.length);
        } else {
            // Nothing to send, length byte=0
            tx_buffer[0] = 0;
        }

        // Trigger the transfer!
        spi_slave_transaction_t transaction = {
            .length = SPI_BUFFER_LEN*8,
            .tx_buffer = tx_buffer,
            .rx_buffer = rx_buffer,
        };

        // printf("Transaction ...\n");
        spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);

        xSemaphoreTake(transfer_done, portMAX_DELAY);

        // printf("Transferred: %d\n", transaction.trans_len);

        // int len  = rx_buffer[0];
        // printf("Rx[%d]", rx_buffer[0]);
        // for (int i=0; i<len; i++) {
        //     printf(" %02x", rx_buffer[i+1]);
        // }
        // printf(" | ");
        // for (int i=len; i<len+10; i++) {
        //     printf(" %02x", rx_buffer[i+1]);
        // }
        // printf("\n");

        // If there is some data received, push the packet in the RX queue!
        if (rx_buffer[0] != 0) {
            packet.length = rx_buffer[0];
            memcpy(packet.data, &rx_buffer[1], packet.length);
            xQueueSend(rx_queue, &packet, portMAX_DELAY);
        }
    }
}

void spi_transport_init() {
    printf("Initializing the SPI transport!\n");

    // Setting up synchronization items
    gap_rtt_event = xSemaphoreCreateBinary();
    tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(spi_transport_packet_t));
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(spi_transport_packet_t));
    transfer_done = xSemaphoreCreateBinary();

    transport_queue_set = xQueueCreateSet(1 + TX_QUEUE_LENGTH);
    xQueueAddToSet(gap_rtt_event, transport_queue_set);
    xQueueAddToSet(tx_queue, transport_queue_set);

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
    xTaskCreate(spi_task, "SPI transport", 10000, NULL, 1, &spi_task_handle);    


    // vTaskDelay(500);

    // xSemaphoreGive(gap_rtt_event);

    // while (1) {
    //     int value = gpio_get_level(GAP_RTT_GPIO);
    //     printf("Pin value: %d\n", value);

    //     vTaskDelay(100);
    // }
}

void spi_transport_send(const spi_transport_packet_t *packet) {
    xQueueSend(tx_queue, packet, portMAX_DELAY);
}

void spi_transport_receive(spi_transport_packet_t *packet) {
    xQueueReceive(rx_queue, packet, portMAX_DELAY);
}
