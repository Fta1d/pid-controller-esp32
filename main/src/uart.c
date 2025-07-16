#include "config.h"
#include "uart.h"
#include "tcp_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART";
static QueueHandle_t uart_queue = NULL;

esp_err_t uart_init(void) {
    ESP_LOGI(TAG, "Initializing UART...");
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver and get the queue
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFF_SIZE * 2, UART_BUFF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized successfully");
    ESP_LOGI(TAG, "Baud rate: %d, TX: GPIO_%d, RX: GPIO_%d", uart_config.baud_rate, UART_TX_PIN, UART_RX_PIN);
    
    return ESP_OK;
}

cmd_id_t uart_parse_command(char *cmd) {
    switch (cmd[0]) {
        case 'S':
            if (cmd[1] == 'H') return SHOOT;
            if (cmd[2] == 'O') return STOP;
            if (cmd[3] == 'X') return SETX;
            if (cmd[3] == 'Y') return SETY;
            if (cmd[0] == 'S' && cmd[3] == ' ') return SET;
            break;

        case 'P':
            return PASS_ENCODER;
            break;

        case 'A':
            return AA_SYS;
            break;

        case 'T':
            return TARGET;
            break;

        case 'C':
            if (cmd[1] == 'R') return CROSS;
            if (cmd[1] == 'A') return CALIBRATE_BACKLASH;
            break;
        
        default:
            break;
    }
    return ERR;
}

void uart_process_input(char *input) {
    TickType_t start_time = xTaskGetTickCount();

    uint16_t len = strcspn(input, "\r\n");

    char parsed_input[64];
    if (len >= sizeof(parsed_input)) len = sizeof(parsed_input) - 1;
    
    memcpy(parsed_input, input, len);
    parsed_input[len] = '\0';

    cmd_id_t cmd_id = uart_parse_command(parsed_input);

    char *cmd = strtok(parsed_input, " ");

    switch (cmd_id) {
        case STOP: {
            xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);
            break;
        }
            
        case SETX: {
            char *value = strtok(NULL, " ");
            char *dir = strtok(NULL, " ");
            float f_value = atoff(value);

            if (value && dir) {
                x_motor.duty = motor_speed_to_duty(f_value);
                x_motor.dir = atoi(dir);

                xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
            } else {
                ESP_LOGE(TAG, "Missing parameters for %s!", cmd);
            }
            break;
        }

        case SETY: {
            char *value = strtok(NULL, " ");
            char *dir = strtok(NULL, " ");
            float f_value = atoff(value);

            if (value && dir) {
                y_motor.duty = motor_speed_to_duty(f_value);
                y_motor.dir = atoi(dir);

                xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_Y);
            } else {
                ESP_LOGE(TAG, "Missing parameters for %s!", cmd);
            }
            break;
        }

        case SET: {
            char *var = strtok(NULL, " ");
            char *val = strtok(NULL, " ");

            if (var[1] == 'P') {
                pid_set_kp(atoff(val));
            } 
            else if (var[1] == 'I') {
                pid_set_ki(atoff(val));
            }
            else if (var[1] == 'D') {
                pid_set_kd(atoff(val));
            }
            else if (strcmp(var, "PREC") == 0) {
                pid_set_aa_precision_threshold(atoff(val));
            } 
            else if (strcmp(var, "FREQ") == 0) {
                pid_set_aa_sys_freq_ms(atoi(val));
            }

            ESP_LOGI(TAG, "%s %s %s", cmd, var, val);

            break;
        }

        case SHOOT: {
            xEventGroupSetBits(motor_control_event_group, MOTOR_SHOOT_EVENT);
            break;
        }

        case PASS_ENCODER: {
            char *value = strtok(NULL, " ");

            motor_set_pass_encoder(atoi(value));
            break;
        }

        case AA_SYS: {
            char *val = strtok(NULL, " ");
            
            if (val) {
                bool state = atoi(val);
                if (!state) {
                    xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);
                }

                pid_set_aa_sys_state(state);
            }
            break;
        }

        case TARGET: {
            char *abscissa = strtok(NULL, " ");
            char *ordinate = strtok(NULL, " ");

            if (abscissa && ordinate) {
                pid_set_aa_target_pos(atoi(abscissa), atoi(ordinate));
            }
            break;
        }

        case CROSS: {
            char *abscissa = strtok(NULL, " ");
            char *ordinate = strtok(NULL, " ");

            if (abscissa && ordinate) {
                pid_set_aa_crosshair_pos(atoi(abscissa), atoi(ordinate));
            }
            break;
        }

        case CALIBRATE_BACKLASH: {
            calculate_backlash();
            break;
        }

        case ERR:
        default:
            break;
    }

    TickType_t end_time = xTaskGetTickCount();
    uint32_t process_time = (end_time - start_time) * portTICK_PERIOD_MS;
    
    if (process_time > 10) { 
        ESP_LOGW(TAG, "Slow processing: %lu ms for '%s'", process_time, input);
    }
}

static void uart_task(void *pvParameters) {
    ESP_LOGI(TAG, "UART event task started");
    uart_write_bytes(UART_NUM, "ACK\n", 4);

    uart_event_t event;
    uint8_t data[UART_READ_BUFFER_SIZE] = {0};
    int index = 0;
    uint8_t *dtmp = (uint8_t*) malloc(UART_READ_BUFFER_SIZE);
    
    while (1) {
        // Waiting for UART event from ISR
        if (xQueueReceive(uart_queue, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    ESP_LOGD(TAG, "[UART DATA]: %d", event.size);
                    
                    // Read all available data
                    int len = uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    
                    if (len > 0) {
                        for (int i = 0; i < len; i++) {
                            uint8_t ch = dtmp[i];
                            
                            if (ch == '\n' || ch == '\r') {
                                if (index > 0) {
                                    data[index] = '\0';
                                    uart_process_input((char *)data);
                                    
                                    index = 0;
                                    memset(data, 0, UART_READ_BUFFER_SIZE);
                                }
                            } else if (index < (UART_READ_BUFFER_SIZE - 1)) {
                                data[index++] = ch;
                            } else {
                                ESP_LOGE(TAG, "UART buffer overflow!");
                                index = 0;
                                memset(data, 0, UART_READ_BUFFER_SIZE);
                            }
                        }
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "HW FIFO Overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring Buffer Full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BREAK:
                    ESP_LOGW(TAG, "RX Break");
                    break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "Parity Error");
                    break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "Frame Error");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown UART event type: %d", event.type);
                    break;
            }
        }
    }
    
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
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

void uart_task_create(void) {
    BaseType_t ret = xTaskCreate(uart_task, "uart_event_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART event task");
    } else {
        ESP_LOGI(TAG, "UART event task created successfully");
    }
}