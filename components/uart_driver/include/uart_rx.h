#ifndef UART_RX_H
#define UART_RX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void telemetry_uart_init(QueueHandle_t data_queue);

#endif 