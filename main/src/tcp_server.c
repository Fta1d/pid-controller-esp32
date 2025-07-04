#include "tcp_server.h"
#include "wifi_ap.h"
#include "motor.h"
#include "pid.h"
#include "trigger.h"
#include "encoder.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "TCP";
static int tcp_server_socket = -1;
static int client_socket = -1;

cmd_id_t tcp_parse_command(char *cmd) {
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
            return CROSS;
            break;
        
        default:
            break;
    }
    return ERR;
}

void process_input(char *input) {
    TickType_t start_time = xTaskGetTickCount();

    uint16_t len = strcspn(input, "\r\n");

    char parsed_input[64];
    if (len >= sizeof(parsed_input)) len = sizeof(parsed_input) - 1;
    
    memcpy(parsed_input, input, len);
    parsed_input[len] = '\0';

    cmd_id_t cmd_id = tcp_parse_command(parsed_input);

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
                ESP_LOGE("TCP", "Missing parameters for %s!", cmd);
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
                ESP_LOGE("TCP", "Missing parameters for %s!", cmd);
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
            else if (var[3] == 'F') {
                pid_set_aa_sys_freq_ms(atoi(val));
            }
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

void tcp_server_task(void *pvParameters) {
    const TickType_t tcp_task_freq = pdMS_TO_TICKS(10);
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (!wifi_is_ap_started()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Waiting for Access Point to start...");
    }
    
    tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(tcp_server_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(tcp_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    if (listen(tcp_server_socket, 4) < 0) {
        ESP_LOGE(TAG, "Failed to listen");
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "TCP Server listening on 192.168.4.1:%d", TCP_PORT);
    ESP_LOGI(TAG, "Ready to accept connections from Python client");
    
    while (1) {
        client_socket = accept(tcp_server_socket, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket >= 0) {
            ESP_LOGI(TAG, "Python client connected");

            const char* welcome = "ESP32 Turret Controller Ready!\n";
            send(client_socket, welcome, strlen(welcome), 0);
            
            uint8_t buffer[512];
            char line_buffer[256];
            int line_pos = 0;

            int flags = fcntl(client_socket, F_GETFL, 0);
            fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);
            
            TickType_t last_wake_time = xTaskGetTickCount();

            while (1) {
                int len = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
                
                if (len > 0) {
                    for (int i = 0; i < len; i++) {
                        char c = buffer[i];

                        if (c == '\n' || c == '\r') {
                            if (line_pos > 0) {
                                line_buffer[line_pos] = '\0';
                                process_input(line_buffer);
                                line_pos = 0; 
                            }
                        } else if (line_pos < sizeof(line_buffer) - 1) {
                            line_buffer[line_pos++] = c;
                        }
                    }
                } else if (len == 0) {
                    ESP_LOGI(TAG, "Client disconnected");
                    break;
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        ESP_LOGI(TAG, "Client disconnected with error: %s", strerror(errno));
                        break;
                    }
                }
                xTaskDelayUntil(&last_wake_time, tcp_task_freq);
            }
            
            close(client_socket);
            client_socket = -1;
        } else {
            ESP_LOGE(TAG, "Accept failed: %s", strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void tcp_server_task_create(void) {
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 15, NULL);
}