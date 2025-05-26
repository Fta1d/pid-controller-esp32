#include "main.h"

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI("I2C", "I2C master initialized successfully");
    return ESP_OK;
}

esp_err_t read_encoder_i2c(uint8_t encoder_addr, int16_t *position) {
    uint8_t data[2];
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Start condition
    i2c_master_start(cmd);
    
    // Write encoder address + write bit
    i2c_master_write_byte(cmd, (encoder_addr << 1) | I2C_MASTER_WRITE, true);
    
    // Write register address (angle high byte)
    i2c_master_write_byte(cmd, ENCODER_REG_ANGLE_HIGH, true);
    
    // Restart condition
    i2c_master_start(cmd);
    
    // Write encoder address + read bit
    i2c_master_write_byte(cmd, (encoder_addr << 1) | I2C_MASTER_READ, true);
    
    // Read high byte
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    
    // Read low byte
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    
    // Stop condition
    i2c_master_stop(cmd);
    
    // Execute command
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Combine high and low bytes (12-bit resolution for AS5600)
        uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
        
        // Convert to signed 16-bit value (adjust scaling as needed)
        // For AS5600: 4096 counts per full rotation (12-bit)
        *position = (int16_t)(raw_angle & 0x0FFF);
        
        ESP_LOGD("I2C", "Encoder 0x%02X: raw=0x%04X, pos=%d", encoder_addr, raw_angle, *position);
    } else {
        ESP_LOGW("I2C", "Failed to read encoder 0x%02X: %s", encoder_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

void configure_uart() {
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
}

void configure_pwm() {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_CLK
    };
    ledc_timer_config(&timer_config);

    ledc_channel_config_t pwm_channels[] = {
        {
            .channel    = X_IN1_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = X_MOTOR_IN1_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },

        {
            .channel    = X_IN2_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = X_MOTOR_IN2_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },

        {
            .channel    = Y_IN1_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = Y_MOTOR_IN1_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },

        {
            .channel    = Y_IN2_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = Y_MOTOR_IN2_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };

    for (size_t i = 0; i < PWM_CHANNELS_NUM; i++) {
        ledc_channel_config(&pwm_channels[i]);
    }

    ledc_fade_func_install(0);
}

void motor_set_state(motor_t *motor, motor_state_t state, uint32_t pwm_value) {
    ledc_channel_t ch1 = motor->in1_channel;
    ledc_channel_t ch2 = motor->in2_channel;
    
    switch (state) {
        case MOTOR_STOP:
            // Free rotation - both channels = 0
            ledc_set_duty(LEDC_MODE, ch1, 0);
            ledc_set_duty(LEDC_MODE, ch2, 0);
            break;
            
        case MOTOR_FORWARD:
            // Move forward - IN1=PWM, IN2=0
            if (pwm_value < MIN_PWM_DUTY) pwm_value = 0;
            if (pwm_value > MAX_PWM_DUTY) pwm_value = MAX_PWM_DUTY;
            
            ledc_set_duty(LEDC_MODE, ch1, pwm_value);
            ledc_set_duty(LEDC_MODE, ch2, 0);
            break;
            
        case MOTOR_REVERSE:
            // Move reverse - IN1=0, IN2=PWM
            if (pwm_value < MIN_PWM_DUTY) pwm_value = 0;
            if (pwm_value > MAX_PWM_DUTY) pwm_value = MAX_PWM_DUTY;
            
            ledc_set_duty(LEDC_MODE, ch1, 0);
            ledc_set_duty(LEDC_MODE, ch2, pwm_value);
            break;
            
        case MOTOR_BRAKE:
            // MAGNETIC BREAK
            ledc_set_duty(LEDC_MODE, ch1, MAX_PWM_DUTY);
            ledc_set_duty(LEDC_MODE, ch2, MAX_PWM_DUTY);
            break;
    }
    
    ledc_update_duty(LEDC_MODE, ch1);
    ledc_update_duty(LEDC_MODE, ch2);
}

void motor_set_pwm(motor_t *motor, float output) {
    if (system_brake_engaged) {
        motor_set_state(motor, MOTOR_BRAKE, 0);
        return;
    }

    if (output > MAX_PWM_DUTY) output = MAX_PWM_DUTY;
    if (output < -MAX_PWM_DUTY) output = -MAX_PWM_DUTY;
    
    uint32_t duty = (uint32_t)fabs(output);
    
    if (duty < MIN_PWM_DUTY) {
        motor_set_state(motor, MOTOR_STOP, 0);
    } else if (output > 0) {
        motor_set_state(motor, MOTOR_FORWARD, duty);
    } else {
        motor_set_state(motor, MOTOR_REVERSE, duty);
    }
}

void software_interrupt_brake(void) {
    ESP_LOGI("BRAKE", "SOFTWARE INTERRUPT: MAGNETIC BRAKE ENGAGED");

    motor_x.pid.active = false;
    motor_y.pid.active = false;

    system_brake_engaged = true;
    motor_set_state(&motor_x, MOTOR_BRAKE, 0);
    motor_set_state(&motor_y, MOTOR_BRAKE, 0);
    
    xSemaphoreGive(pid_wake_semaphore);
}

void software_interrupt_release_brake(void) {
    ESP_LOGI("BRAKE", "SOFTWARE INTERRUPT: BRAKE RELEASED");

    system_brake_engaged = false;
    motor_set_state(&motor_x, MOTOR_STOP, 0);
    motor_set_state(&motor_y, MOTOR_STOP, 0);
}

void software_interrupt_move_x(int16_t target_position) {
    ESP_LOGI("ISR", "SOFTWARE INTERRUPT: Move X to %d", target_position);

    if (system_brake_engaged) {
        software_interrupt_release_brake();
    }

    // Read current encoder position
    int16_t current_pos;
    if (read_encoder_i2c(motor_x.encoder_i2c_addr, &current_pos) == ESP_OK) {
        motor_x.encoder_pos = current_pos;
    }

    motor_x.pid.target_pos = target_position;
    motor_x.pid.current_pos = motor_x.encoder_pos;
    motor_x.pid.active = true;
    motor_x.pid.integral = 0;
    motor_x.pid.prev_err = 0;
    motor_x.pid.last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    xEventGroupSetBits(pid_interrupt_group, PID_INTERRUPT_X_MOVE);
    xSemaphoreGive(pid_wake_semaphore);
}

void software_interrupt_move_y(int16_t target_position) {
    ESP_LOGI("ISR", "SOFTWARE INTERRUPT: Move Y to %d", target_position);

    if (system_brake_engaged) {
        software_interrupt_release_brake();
    }
    
    // Read current encoder position
    int16_t current_pos;
    if (read_encoder_i2c(motor_y.encoder_i2c_addr, &current_pos) == ESP_OK) {
        motor_y.encoder_pos = current_pos;
    }
    
    motor_y.pid.target_pos = target_position;
    motor_y.pid.current_pos = motor_y.encoder_pos;
    motor_y.pid.active = true;
    motor_y.pid.integral = 0;
    motor_y.pid.prev_err = 0;
    motor_y.pid.last_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    xEventGroupSetBits(pid_interrupt_group, PID_INTERRUPT_Y_MOVE);
    xSemaphoreGive(pid_wake_semaphore);
}

void software_interrupt_stop(void) {
    ESP_LOGI("ISR", "SOFTWARE INTERRUPT: STOP (free coast)");
    
    motor_x.pid.active = false;
    motor_y.pid.active = false;

    if (!system_brake_engaged) {
        motor_set_state(&motor_x, MOTOR_STOP, 0);
        motor_set_state(&motor_y, MOTOR_STOP, 0);
    }
    
    xEventGroupSetBits(pid_interrupt_group, PID_INTERRUPT_ALL_STOP);
    xSemaphoreGive(pid_wake_semaphore);
}

float calculate_pid(motor_t *motor) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (current_time - motor->pid.last_time) / 1000.0;
    if (dt <= 0) dt = 0.01;
    
    float error = motor->pid.target_pos - motor->pid.current_pos;
    
    float proportional = Kp * error;
    
    motor->pid.integral += error * dt;
    if (motor->pid.integral > integral_limit) motor->pid.integral = integral_limit;
    if (motor->pid.integral < -integral_limit) motor->pid.integral = -integral_limit;
    float integral = Ki * motor->pid.integral;
    
    float derivative = Kd * (error - motor->pid.prev_err) / dt;
    
    motor->pid.prev_err = error;
    motor->pid.last_time = current_time;
    
    return proportional + integral + derivative;
}

void process_cmd(char *command) {
    uint16_t len = strcspn(command, "\r\n");
    char *parsed_cmd = (char *)malloc(len + 1);
    char error_msg[] = "> Invalid command!\r\n> ";
    
    memcpy(parsed_cmd, command, len);
    parsed_cmd[len] = '\0';
    
    ESP_LOGI("INFO", "Parsing command: %s", parsed_cmd);

    if (strcmp(parsed_cmd, "brake") == 0) {
        software_interrupt_brake();
        uart_write_bytes(UART_NUM, "> Magnetic brake engaged!\r\n> ", 31);
    }
    else if (strcmp(parsed_cmd, "release") == 0) {
        software_interrupt_release_brake();
        uart_write_bytes(UART_NUM, "> Brake released!\r\n> ", 23);
    }
    else if (strcmp(parsed_cmd, "stop") == 0) {
        software_interrupt_stop();
        uart_write_bytes(UART_NUM, "> Motors stopped\r\n> ", 20);
    }
    else if (strcmp(parsed_cmd, "status") == 0) {
        char status[200];
        const char* brake_status = system_brake_engaged ? "BRAKE" : "FREE";
        
        snprintf(status, sizeof(status), 
                "> X: pos=%d->%d (%s) | Y: pos=%d->%d (%s) | System: %s\r\n> ",
                motor_x.encoder_pos, motor_x.pid.target_pos, motor_x.pid.active ? "ON" : "OFF",
                motor_y.encoder_pos, motor_y.pid.target_pos, motor_y.pid.active ? "ON" : "OFF",
                brake_status);
        uart_write_bytes(UART_NUM, status, strlen(status));
    }
    else if (strchr(parsed_cmd, ':')) {
        char *motor = strtok(parsed_cmd, ":");
        char *pos_str = strtok(NULL, ":");
        
        if (motor && pos_str) {
            int16_t pos = (int16_t)atoi(pos_str);
            
            if (strcmp(motor, "x") == 0) {
                software_interrupt_move_x(pos);  
                char response[50];
                snprintf(response, sizeof(response), "> X -> %d\r\n> ", pos);
                uart_write_bytes(UART_NUM, response, strlen(response));
            }
            else if (strcmp(motor, "y") == 0) {
                software_interrupt_move_y(pos); 
                char response[50];
                snprintf(response, sizeof(response), "> Y -> %d\r\n> ", pos);
                uart_write_bytes(UART_NUM, response, strlen(response));
            }
            else {
                uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
            }
        } else {
            uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
        }
    } else {
        uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
    }
    
    free(parsed_cmd);
}

void pid_controller_task(void *pvParameters) {
    EventBits_t interrupt_bits;
    
    ESP_LOGI("INFO", "PID Controller sleeping, waiting for software interrupts...");
    
    while (1) {
        if (xSemaphoreTake(pid_wake_semaphore, portMAX_DELAY) == pdTRUE) {

            while (xSemaphoreTake(pid_wake_semaphore, 0) == pdTRUE) {
                // Clearing accumulated semaphore signals
            }

            interrupt_bits = xEventGroupGetBits(pid_interrupt_group);
            
            ESP_LOGI("INFO", "PID Controller WOKEN by software interrupt! Bits: 0x%02lX", interrupt_bits);

            if (interrupt_bits == 0) {
                ESP_LOGI("INFO", "No active interrupt bits, going back to sleep...");
                continue;
            }

            if (interrupt_bits & PID_INTERRUPT_X_MOVE) {
                ESP_LOGI("INFO", "Processing X MOVE interrupt");
            }
            if (interrupt_bits & PID_INTERRUPT_Y_MOVE) {
                ESP_LOGI("INFO", "Processing Y MOVE interrupt");
            }
            if (interrupt_bits & PID_INTERRUPT_ALL_STOP) {
                ESP_LOGI("INFO", "Processing EMERGENCY STOP interrupt");
                xEventGroupClearBits(pid_interrupt_group, PID_INTERRUPT_ALL_STOP);
                continue;
            }

            while (motor_x.pid.active || motor_y.pid.active) {

                if (motor_x.pid.active) {
                    // Read encoder position only when motor is active
                    int16_t current_pos;
                    if (read_encoder_i2c(motor_x.encoder_i2c_addr, &current_pos) == ESP_OK) {
                        motor_x.encoder_pos = current_pos;
                        motor_x.pid.current_pos = current_pos;
                    } else {
                        ESP_LOGW("PID", "Failed to read X encoder, using last known position");
                        motor_x.pid.current_pos = motor_x.encoder_pos;
                    }
                    
                    float error_x = motor_x.pid.target_pos - motor_x.pid.current_pos;
                    
                    if (fabs(error_x) <= POSITION_TOLERANCE) {
                        motor_x.pid.active = false;
                        motor_set_pwm(&motor_x, 0);
                        ESP_LOGI("INFO", "Motor X reached target: %d", motor_x.pid.target_pos);
                    } else {
                        float output_x = calculate_pid(&motor_x);
                        motor_set_pwm(&motor_x, output_x);
                    }
                }
                
                if (motor_y.pid.active) {
                    // Read encoder position only when motor is active
                    int16_t current_pos;
                    if (read_encoder_i2c(motor_y.encoder_i2c_addr, &current_pos) == ESP_OK) {
                        motor_y.encoder_pos = current_pos;
                        motor_y.pid.current_pos = current_pos;
                    } else {
                        ESP_LOGW("PID", "Failed to read Y encoder, using last known position");
                        motor_y.pid.current_pos = motor_y.encoder_pos;
                    }
                    
                    float error_y = motor_y.pid.target_pos - motor_y.pid.current_pos;
                    
                    if (fabs(error_y) <= POSITION_TOLERANCE) {
                        motor_y.pid.active = false;
                        motor_set_pwm(&motor_y, 0);
                        ESP_LOGI("INFO", "Motor Y reached target: %d", motor_y.pid.target_pos);
                    } else {
                        float output_y = calculate_pid(&motor_y);
                        motor_set_pwm(&motor_y, output_y);
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(PID_UPDATE_PERIOD));
            }
            
            xEventGroupClearBits(pid_interrupt_group, 0xFF);
            ESP_LOGI("INFO", "All motors stopped. PID Controller going to sleep...");
        }
    }
}

void uart_task(void *pvParameters) {
    const char* welcome_msg = "\r\n=== ESP32 PWM Controller with I2C Encoders ===\r\n> ";
    const char* help = "Commands:\r\n"
                    "  x:pos, y:pos - move motors (with I2C encoder feedback)\r\n"
                    "  brake - engage magnetic brake\r\n"
                    "  release - release brake\r\n"
                    "  stop - stop motors (free coast)\r\n"
                    "  status - show status\r\n> ";
    uart_write_bytes(UART_NUM, welcome_msg, strlen(welcome_msg));
    uart_write_bytes(UART_NUM, help, strlen(help));

    uint8_t data[UART_BUFF_SIZE];

    while (1) {
        size_t len = uart_read_bytes(UART_NUM, data, UART_BUFF_SIZE - 1, pdMS_TO_TICKS(10));
        data[len] = '\0';

        if (len) {
            ESP_LOGI("UART", "%s", (char *)data);
            process_cmd((char *)data);
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void app_main(void) {
    // Initialize I2C first
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return;
    }

    configure_pwm();
    configure_uart();

    pid_interrupt_group = xEventGroupCreate();
    pid_wake_semaphore = xSemaphoreCreateBinary();

    motor_x.pid.active = false;
    motor_y.pid.active = false;

    // Read initial encoder positions
    int16_t initial_pos_x, initial_pos_y;
    if (read_encoder_i2c(motor_x.encoder_i2c_addr, &initial_pos_x) == ESP_OK) {
        motor_x.encoder_pos = initial_pos_x;
        ESP_LOGI("MAIN", "Initial X encoder position: %d", initial_pos_x);
    } else {
        ESP_LOGW("MAIN", "Failed to read initial X encoder position");
    }
    
    if (read_encoder_i2c(motor_y.encoder_i2c_addr, &initial_pos_y) == ESP_OK) {
        motor_y.encoder_pos = initial_pos_y;
        ESP_LOGI("MAIN", "Initial Y encoder position: %d", initial_pos_y);
    } else {
        ESP_LOGW("MAIN", "Failed to read initial Y encoder position");
    }

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL);
    xTaskCreatePinnedToCore(pid_controller_task, "pid_controller", 8192, NULL, 5, NULL, 1);

}