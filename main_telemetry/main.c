#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "telemetry_data.h"
#include "nvs_flash.h"
#include "uart_rx.h"
#include "udp_server.h"
QueueHandle_t q_handle;

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    q_handle = xQueueCreate(20, sizeof(FlyData_t));

    telemetry_uart_init(q_handle);
    telemetry_wifi_init(q_handle);
}
