#ifndef UART_COMM_H
#define UART_COMM_H

#include "motor.h"
#include "pid.h"
#include "trigger.h"
#include "backlash_compensator.h"

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// UART Configuration
#define UART_BUFF_SIZE      1024
#define UART_READ_BUFFER_SIZE 256

// UART Event Queue size
#define UART_QUEUE_SIZE     20

// Command IDs
typedef enum {
    SETX = 0,
    SETY,
    STOP,
    SHOOT,
    PASS_ENCODER,
    TARGET,
    CROSS,
    AA_SYS,
    SET,
    CALIBRATE_BACKLASH,
    ERR
} cmd_id_t;

// Function prototypes
esp_err_t uart_init(void);
cmd_id_t uart_parse_command(char *cmd);
void uart_process_command(char *input);
void uart_task_create(void);
esp_err_t uart_send_message(const char* message);

#endif // UART_COMM_H