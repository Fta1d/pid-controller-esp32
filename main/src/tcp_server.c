#include "tcp_server.h"
#include "wifi_ap.h"
#include "motor.h"
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
static bool analog_mode_active = false;

void reset_current_analog_state(void) {
    analog_state.x_speed = 0;
    analog_state.y_speed = 0;
    analog_state.x_direction = false;
    analog_state.y_direction = false;
    analog_state.x_active = false;
    analog_state.y_active = false;
}

// void parse_analog_command(const char *input) {
//     ESP_LOGI(TAG, "Parsing analog command: '%s'", input);
    
//     controls.up_pressed = false;
//     controls.down_pressed = false;
//     controls.left_pressed = false;
//     controls.right_pressed = false;
    
//     if (strcmp(input, "SHOOT") == 0) {
//         trigger_shoot();
//         return;
//     }
    
//     reset_current_analog_state();
    
//     if (strcmp(input, "STOP") == 0) {
//         return;
//     }

//     if (strncmp(input, "SET", 3) == 0) {
//         const char *ptr = input + 3;

//         char axis = *ptr;
//         if (axis != 'X' && axis != 'Y') {
//             ESP_LOGW(TAG, "Invalid axis: %c", axis);
//             return;
//         }
//         ptr++; 

//         while (*ptr == ' ') ptr++;

//         float normalized_freq = 0.0f;
//         char *endptr;
//         normalized_freq = strtof(ptr, &endptr);
        
//         if (endptr == ptr) {
//             ESP_LOGW(TAG, "Invalid frequency value");
//             return;
//         }

//         if (normalized_freq < 0.0f || normalized_freq > 1.0f) {
//             ESP_LOGW(TAG, "Frequency out of range: %.3f", normalized_freq);
//             return;
//         }
        
//         ptr = endptr;

//         while (*ptr == ' ') ptr++;

//         int direction = atoi(ptr);
//         if (direction != 0 && direction != 1) {
//             ESP_LOGW(TAG, "Invalid direction: %d", direction);
//             return;
//         }

//         if (axis == 'X') {
//             analog_state.x_active = (normalized_freq > 0.0f); 
//             analog_state.x_speed = normalized_freq; 
//             analog_state.x_direction = (direction == 1);
//             ESP_LOGI(TAG, "X axis: speed=%.3f, direction=%d", normalized_freq, direction);
//         } else if (axis == 'Y') {
//             analog_state.y_active = (normalized_freq > 0.0f); 
//             analog_state.y_speed = normalized_freq; 
//             analog_state.y_direction = (direction == 1);
//             ESP_LOGI(TAG, "Y axis: speed=%.3f, direction=%d", normalized_freq, direction);
//         }
        
//         return;
//     }
// }

cmd_id_t tcp_parse_command(char *cmd) {
    switch (cmd[0]) {
        case 'S':
            if (cmd[1] == 'H') return SHOOT;
            if (cmd[2] == 'O') return STOP;
            if (cmd[3] == 'X') return SETX;
            if (cmd[3] == 'Y') return SETY;
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
                analog_state.x_speed = motor_speed_to_duty(f_value);
                analog_state.x_direction = atoi(dir);

                xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
            } else {
                ESP_LOGE("TCP", "Missing parameters for SETX!");
            }
            break;
        }

        case SETY: {
            char *value = strtok(NULL, " ");
            char *dir = strtok(NULL, " ");
            float f_value = atoff(value);

            if (value && dir) {
                analog_state.y_speed = motor_speed_to_duty(f_value);
                analog_state.y_direction = atoi(dir);

                xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_Y);
            } else {
                ESP_LOGE("TCP", "Missing parameters for SETY!");
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

            break;
        }

        case TARGET: {

            break;
        }

        case CROSS: {

            break;
        }

        case ERR:
        default:
            break;
    }

    // bool is_analog = false;
    // if (len == 4 && strcmp(parsed_input, "STOP") == 0) {
    //     is_analog = true;
    // } else if (len == 5 && strcmp(parsed_input, "SHOOT") == 0) {
    //     is_analog = true;
    // } else if (len > 3 && strncmp(parsed_input, "SET", 3) == 0) {
    //     is_analog = true;
    // }
    
    // if (is_analog) {
    //     analog_mode_active = true;
    //     motor_set_analog_mode(true);
    //     parse_analog_command(parsed_input);

    //     if (analog_state.x_active) {
    //         motor_set_speed_analog(&motor_x, analog_state.x_speed, analog_state.x_direction);
    //         x_state = analog_state.x_direction ? MOTOR_FORWARD : MOTOR_BACKWARD;
    //     } else if (x_state != MOTOR_STOPPED) { 
    //         motor_set_speed_analog(&motor_x, 0, true);
    //         x_state = MOTOR_STOPPED;
    //     }
        
    //     if (analog_state.y_active) {
    //         bool can_move = true;
    //         if (!motor_get_pass_encoder()) {
    //             if (!analog_state.y_direction && turret_pos.y_angle >= MAX_Y_ANGLE) {
    //                 can_move = false;
    //             } else if (analog_state.y_direction && turret_pos.y_angle <= MIN_Y_ANGLE) {
    //                 can_move = false;
    //             }
    //         }

    //         if (can_move) {
    //             motor_set_speed_analog(&motor_y, analog_state.y_speed, analog_state.y_direction);
    //             y_state = analog_state.y_direction ? MOTOR_FORWARD : MOTOR_BACKWARD;
    //         } else if (y_state != MOTOR_STOPPED) {  
    //             motor_set_speed_analog(&motor_y, 0, true);
    //             y_state = MOTOR_STOPPED;
    //         }
    //     } else if (y_state != MOTOR_STOPPED) { 
    //         motor_set_speed_analog(&motor_y, 0, true);
    //         y_state = MOTOR_STOPPED;
    //     }
    // } else {
    //     analog_mode_active = false;
    //     motor_set_analog_mode(false);
    //     char *ptr = parsed_input;

    //     if (*ptr == 0x39 ) {
    //         DUTY = MAX_PWM_DUTY;
    //         ESP_LOGI(TAG, "Duty set to MAX: %d", DUTY);
    //     }

    //     if (*ptr == 0x31) {
    //         DUTY = MIN_PWM_DUTY;
    //         ESP_LOGI(TAG, "Duty set to MIN: %d", DUTY);
    //     }

    //     if (*ptr == '+') {
    //         uint16_t temp_duty = DUTY + DUTY_STEP;
    //         temp_duty > MAX_PWM_DUTY ? (DUTY = MAX_PWM_DUTY) : (DUTY = temp_duty);
    //         ESP_LOGI(TAG, "Duty increased to: %d", (int)DUTY);
    //     }
    //     if (*ptr == '-') {
    //         uint16_t temp_duty = DUTY - DUTY_STEP;
    //         temp_duty < MIN_PWM_DUTY ? (DUTY = MIN_PWM_DUTY) : (DUTY = temp_duty);
    //         ESP_LOGI(TAG, "Duty decreased to: %d", (int)DUTY);
    //     }

    //     if (*ptr == 'n') {
    //         uint16_t temp_shoot_time = shoot_time - 10;
    //         temp_shoot_time <= MIN_SHOOT_TIME ? (shoot_time = MIN_SHOOT_TIME) : (shoot_time = temp_shoot_time);
    //         ESP_LOGI(TAG, "Shoot time decreased to: %d ms", (int)shoot_time);
    //     }
    //     if (*ptr == 'm') {
    //         uint16_t temp_shoot_time = shoot_time + 10;
    //         temp_shoot_time >= MAX_SHOOT_TIME ? (shoot_time = MAX_SHOOT_TIME) : (shoot_time = temp_shoot_time);
    //         ESP_LOGI(TAG, "Shoot time increased to: %d ms", (int)shoot_time);
    //     }

    //     if (*ptr == 'p') {
    //         motor_set_pass_encoder(!motor_get_pass_encoder());
    //         ESP_LOGI(TAG, "Pass encoder toggled: %d", motor_get_pass_encoder());
    //     }

    //     if (*ptr == 0x20) {
    //         ESP_LOGI(TAG, "Shoot fired!");
    //         trigger_shoot();
    //     }
        
    //     if (*ptr == 'Z') {
    //         ESP_LOGI(TAG, "Calibrating both axes...");
    //         if (encoder_calibrate_turret() == ESP_OK) {
    //             ESP_LOGI(TAG, "Dual calibration successful");
    //         } else {
    //             ESP_LOGI(TAG, "Calibration failed");
    //         }
    //     }
    // }

    TickType_t end_time = xTaskGetTickCount();
    uint32_t process_time = (end_time - start_time) * portTICK_PERIOD_MS;
    
    if (process_time > 10) { 
        ESP_LOGW(TAG, "Slow processing: %lu ms for '%s'", process_time, input);
    }
}

void tcp_server_task(void *pvParameters) {
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
            
            while (1) {
                int len = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
                
                if (len <= 0) {
                    ESP_LOGI(TAG, "Client disconnected");
                    break;
                }

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
            }
            
            close(client_socket);
            client_socket = -1;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}