#include "uart.h"
#include "tcp_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART";

esp_err_t uart_init(void) {
    ESP_LOGI(TAG, "Initializing UART communication...");
    
    uart_config_t uart_config = {
        .baud_rate           = 115200,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized successfully");
    ESP_LOGI(TAG, "Baud rate: 115200");
    ESP_LOGI(TAG, "TX pin: GPIO_17, RX pin: GPIO_16");
    
    return ESP_OK;
}

void uart_task(void *pvParameters) {
    const char* welcome_msg = "\r\n=== ESP32 PWM Controller ===\r\n> ";
    
    uart_write_bytes(UART_NUM, welcome_msg, strlen(welcome_msg));
    ESP_LOGI(TAG, "UART task started, welcome message sent");

    uint8_t data[UART_BUFF_SIZE];

    while (1) {
        size_t len = uart_read_bytes(UART_NUM, data, UART_BUFF_SIZE - 1, pdMS_TO_TICKS(10));
        
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "UART received: %s", (char *)data);

            process_input((char *)data);

            const char* prompt = "> ";
            uart_write_bytes(UART_NUM, prompt, strlen(prompt));
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t uart_send_message(const char* message) {
    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(message);
    int bytes_written = uart_write_bytes(UART_NUM, message, len);
    
    if (bytes_written == len) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send complete message via UART");
        return ESP_FAIL;
    }
}

esp_err_t uart_send_status(float x_angle, float y_angle) {
    char status_msg[128];
    snprintf(status_msg, sizeof(status_msg), 
             "Status: X=%.2f° Y=%.2f° PWM=%d ShootTime=%dms\r\n", 
             x_angle, y_angle, DUTY, shoot_time);
    
    return uart_send_message(status_msg);
}

void uart_flush_rx_buffer(void) {
    uart_flush_input(UART_NUM);
    ESP_LOGD(TAG, "RX buffer flushed");
}