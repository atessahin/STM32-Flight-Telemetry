#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define ESP_WIFI_SSID      "TELEMETRY_AP"
#define ESP_WIFI_PASS      "12345678"
#define ESP_WIFI_CHANNEL   1
#define MAX_STA_CONN       4
#define UDP_PORT           3333
void telemetry_wifi_init(QueueHandle_t data_queue);