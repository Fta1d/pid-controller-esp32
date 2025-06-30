#ifndef UART_COMM_H
#define UART_COMM_H

#include "config.h"
#include "esp_err.h"

esp_err_t uart_init(void);

void uart_task(void *pvParameters);

esp_err_t uart_send_message(const char* message);
esp_err_t uart_send_status(float x_angle, float y_angle);

void uart_flush_rx_buffer(void);

#endif // UART_COMM_H