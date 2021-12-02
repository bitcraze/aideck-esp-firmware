#include "router.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "spi_transport.h"
#include "uart_transport.h"

static spi_transport_packet_t gap8_rxp;
static spi_transport_packet_t gap8_txp;
static uart_transport_packet_t cf_rxp;
static uart_transport_packet_t cf_txp;

static void router_from_gap8(void* _param) {

    while(1) {
      printf("Waiting for packet from GAP8\n");
      // Wait for incoming packet from GAP8
      spi_transport_receive(&gap8_rxp);
      printf("Got packet from GAP8\n");
      // We need to split up the package here!
      cf_txp.length = gap8_rxp.length;
      memcpy(cf_txp.data, gap8_rxp.data, cf_txp.length);

      printf("Have packet for CF, len=%u\n", cf_txp.length);

      uart_transport_send(&cf_txp);
      
    }
}

static void router_from_crazyflie(void* _param) {

    while(1) {
      uart_transport_receive(&cf_rxp);

      // Right now we're always sending to GAP8 (which always has higher MTU)
      gap8_txp.length = cf_rxp.length;
      memcpy(gap8_txp.data, cf_rxp.data, cf_rxp.length);

      spi_transport_send(&gap8_txp);
    }
}


void router_init() {
  xTaskCreate(router_from_gap8, "Router from GAP8", 10000, NULL, 1, NULL);
  xTaskCreate(router_from_crazyflie, "Router from CF", 10000, NULL, 1, NULL);
}