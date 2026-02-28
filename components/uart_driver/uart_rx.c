#include <string.h>
#include "uart_rx.h"
#include "telemetry_data.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "UART_RX";
static QueueHandle_t uart_queue_handle = NULL;

void TelemetryRX_Task(void *arg)
{
    QueueHandle_t q_arg = (QueueHandle_t)arg;
    
    FlyData_t tempPacket; 

    uint8_t* ptr = (uint8_t*)&tempPacket;
    
    uint8_t bufferLocal[sizeof(FlyData_t)];

    ESP_LOGI(TAG, "UART Dinleme Basladi...");
    
    while (1)
    {
        int len = uart_read_bytes(UART_NUM_2, &bufferLocal[0], 1, portMAX_DELAY);
        if (len > 0 && bufferLocal[0] == 0xCD)
        {
            len = uart_read_bytes(UART_NUM_2, &bufferLocal[1], 1, portMAX_DELAY);
            if (len > 0 && bufferLocal[1] == 0xAB)
            {
                len = uart_read_bytes(UART_NUM_2, &bufferLocal[2], sizeof(FlyData_t) - 2, portMAX_DELAY);
                if(len == sizeof(FlyData_t) - 2)
                {

                    uint8_t xor_val = 0;
                    for (int i = 0; i < sizeof(bufferLocal)-1; i++)
                    {
                         xor_val ^= bufferLocal[i];
                    }

                    if (xor_val == bufferLocal[sizeof(FlyData_t) - 1])
                    {
                        for (int i = 0; i < sizeof(FlyData_t); i++)
                        {
                            ptr[i] = bufferLocal[i];
                        }
                        if(xQueueSend(q_arg, &tempPacket, 0) == pdTRUE)
                        {
                           // succes
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Kuyruk DOLU! Paket atildi.");
                        }
                    }
                    else 
                    {
                        ESP_LOGE(TAG, "Checksum Hatasi! Hsp:%d Gelen:%d", xor_val, bufferLocal[sizeof(FlyData_t)-1]);
                    } 
                }
            }
        }
    }
}

void telemetry_uart_init(QueueHandle_t data_queue)
{
    uart_queue_handle = data_queue;

    uart_config_t config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0));

    xTaskCreate(TelemetryRX_Task, "uart_rx_task", 4096, (void*)uart_queue_handle, 5, NULL);
    
    ESP_LOGI(TAG, "UART Init Tamamlandi.");
}