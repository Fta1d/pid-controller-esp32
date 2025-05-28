#include "main.h"
#include "as5600.h"

turret_position_t turret_pos = {0};
as5600_handle_t x_encoder = NULL;
as5600_handle_t y_encoder = NULL;

static bool wifi_ap_started = false;
static int connected_clients = 0;
static int tcp_server_socket = -1;
static int client_socket = -1;

static void wifi_ap_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        connected_clients++;
        ESP_LOGI("WiFi-AP", "Client connected. Total clients: %d", connected_clients);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        connected_clients--;
        ESP_LOGI("WiFi-AP", "Client disconnected. Total clients: %d", connected_clients);
    } else if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI("WiFi-AP", "Access Point started successfully!");
        wifi_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI("WiFi-AP", "Access Point stopped");
        wifi_ap_started = false;
    }
}

esp_err_t wifi_init_ap(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);        // IP 
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);        // Gateway
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); 
    
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_ap_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    if (strlen(WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("WiFi-AP", "WiFi Access Point initialized");
    ESP_LOGI("WiFi-AP", "SSID: %s", WIFI_AP_SSID);
    ESP_LOGI("WiFi-AP", "Password: %s", WIFI_AP_PASS);
    ESP_LOGI("WiFi-AP", "IP Address: 192.168.4.1");
    ESP_LOGI("WiFi-AP", "Connect your device and use IP: 192.168.4.1:%d", TCP_PORT);

    return ESP_OK;
}

esp_err_t init_dual_encoders(void) {
    ESP_LOGI("encoder", "Initializing dual AS5600 encoders...");
    
    // ========== X ENCODER (I2C0) ==========
    as5600_config_t x_config = {
        .i2c_port = X_ENCODER_I2C_PORT,
        .scl_pin = X_ENCODER_SCL_PIN,
        .sda_pin = X_ENCODER_SDA_PIN,
        .clk_speed = I2C_FREQ_HZ,
        .enable_pullups = true
    };
    
    x_encoder = as5600_init(&x_config);
    // if (x_encoder == NULL) {
    //     ESP_LOGE("encoder", "Failed to initialize X encoder (I2C0)");
    //     return ESP_FAIL;
    // }
    
    ESP_LOGI("encoder", "X encoder initialized on I2C0 (SCL=%d, SDA=%d)", 
             X_ENCODER_SCL_PIN, X_ENCODER_SDA_PIN);

    if (as5600_is_connected(x_encoder) == ESP_OK) {
        ESP_LOGI("encoder", "X encoder connection verified");
    } else {
        ESP_LOGW("encoder", "X encoder connection check failed");
    }
    
    // ========== Y ENCODER (I2C1) ==========
    as5600_config_t y_config = {
        .i2c_port = Y_ENCODER_I2C_PORT,
        .scl_pin = Y_ENCODER_SCL_PIN,
        .sda_pin = Y_ENCODER_SDA_PIN,
        .clk_speed = I2C_FREQ_HZ,
        .enable_pullups = true
    };
    
    y_encoder = as5600_init(&y_config);
    // if (y_encoder == NULL) {
    //     ESP_LOGE("encoder", "Failed to initialize Y encoder (I2C1)");
    //     as5600_deinit(x_encoder);
    //     x_encoder = NULL;
    //     return ESP_FAIL;
    // }
    
    ESP_LOGI("encoder", "Y encoder initialized on I2C1 (SCL=%d, SDA=%d)", 
             Y_ENCODER_SCL_PIN, Y_ENCODER_SDA_PIN);
    
    if (as5600_is_connected(y_encoder) == ESP_OK) {
        ESP_LOGI("encoder", "Y encoder connection verified");
    } else {
        ESP_LOGW("encoder", "Y encoder connection check failed");
    }
    
    ESP_LOGI("encoder", "Both encoders initialized successfully!");

    ESP_LOGI("encoder", "=== X ENCODER DIAGNOSTICS ===");
    as5600_print_diagnostics(x_encoder);
    
    ESP_LOGI("encoder", "=== Y ENCODER DIAGNOSTICS ===");
    as5600_print_diagnostics(y_encoder);
    
    return ESP_OK;
}

esp_err_t read_x_encoder(float *angle) {
    if (x_encoder == NULL) {
        ESP_LOGE("encoder", "X encoder not initialized");
        return ESP_FAIL;
    }
    
    return as5600_read_angle_degrees_sliding(x_encoder, angle, 5);
}

esp_err_t read_y_encoder(float *angle) {
    if (y_encoder == NULL) {
        ESP_LOGE("encoder", "Y encoder not initialized");
        return ESP_FAIL;
    }
    
    return as5600_read_angle_degrees_sliding(y_encoder, angle, 5);
}

esp_err_t calibrate_turret(void) {
    if (x_encoder == NULL || y_encoder == NULL) {
        ESP_LOGE("turret", "Encoders not initialized");
        return ESP_FAIL;
    }
    
    ESP_LOGI("turret", "Starting dual encoder calibration...");
    
    esp_err_t x_ret = as5600_set_current_as_zero(x_encoder);
    esp_err_t y_ret = as5600_set_current_as_zero(y_encoder);
    
    if (x_ret == ESP_OK && y_ret == ESP_OK) {
        turret_pos.x_angle = 0.0f;
        turret_pos.y_angle = 0.0f;
        ESP_LOGI("turret", "Dual calibration complete - both axes zeroed");
        return ESP_OK;
    } else {
        ESP_LOGE("turret", "Calibration failed - X: %s, Y: %s", 
                esp_err_to_name(x_ret), esp_err_to_name(y_ret));
        return ESP_FAIL;
    }
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

void configure_trigger() {
    gpio_config_t gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = 1ULL << TRIGGER_PIN
    };

    gpio_config(&gpio_conf);
}

void shoot() {
    gpio_set_level(TRIGGER_PIN, 1);
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = MS_TO_US(100), 
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false, 
    };

    gptimer_set_alarm_action(shot_timer, &alarm_config);
    gptimer_set_raw_count(shot_timer, 0);
    gptimer_start(shot_timer);
}

static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gpio_set_level(TRIGGER_PIN, 0);
    gptimer_stop(timer);
    return false; 
}

void init_shot_timer() {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    gptimer_new_timer(&timer_config, &shot_timer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    gptimer_register_event_callbacks(shot_timer, &cbs, NULL);
    
    gptimer_enable(shot_timer);
}

void motor_control_task(void *pvParameters) {
    motor_state_t y_state = MOTOR_STOPPED;
    motor_state_t x_state = MOTOR_STOPPED;

    if (init_dual_encoders() != ESP_OK) {
        ESP_LOGE("motor", "Failed to initialize encoders");
        vTaskDelete(NULL);
        return;
    }

    turret_pos.x_angle = 0.0f;
    turret_pos.y_angle = 0.0f;

    while (1) {
        bool up = atomic_load_explicit(&controls.up_pressed, memory_order_acquire);
        bool down = atomic_load_explicit(&controls.down_pressed, memory_order_acquire);
        bool left = atomic_load_explicit(&controls.left_pressed, memory_order_acquire);
        bool right = atomic_load_explicit(&controls.right_pressed, memory_order_acquire);
        
        float x_raw_angle, y_raw_angle;
        
        // if (as5600_read_angle_degrees_sliding(x_encoder, &x_raw_angle, 20) == ESP_OK) {
        //     turret_pos.x_angle = x_raw_angle;
        // }
        
        // if (as5600_read_angle_degrees_sliding(y_encoder, &y_raw_angle, 20) == ESP_OK) {
        //     turret_pos.y_angle = y_raw_angle;
        // }

        // === Y AXIS ===
        if (up && !down) {
            if (y_state != MOTOR_FORWARD) {
                ESP_LOGI("motor", "Y UP: %.2f°", turret_pos.y_angle);

                ledc_set_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL, 0);

                ledc_update_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL);

                ledc_set_fade_with_time(LEDC_MODE, Y_IN1_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
                ledc_fade_start(LEDC_MODE, Y_IN1_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

                y_state = MOTOR_FORWARD;
            }
        } else if (down && !up) {
            if (y_state != MOTOR_BACKWARD) {
                ESP_LOGI("motor", "Y DOWN: %.2f°", turret_pos.y_angle);

                ledc_set_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL, 0);

                ledc_update_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL);

                ledc_set_fade_with_time(LEDC_MODE, Y_IN2_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
                ledc_fade_start(LEDC_MODE, Y_IN2_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

                y_state = MOTOR_BACKWARD;
            }
        } else {
            if (y_state == MOTOR_FORWARD || y_state == MOTOR_BACKWARD) {
                ledc_set_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL);

                y_state = MOTOR_BRAKING;
            }
        }
        
        // === X AXIS ===
        if (left && !right) {
            if (x_state != MOTOR_FORWARD) {
                ESP_LOGI("motor", "X LEFT: %.2f°", turret_pos.x_angle);

                ledc_set_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL, 0);

                ledc_update_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL);

                ledc_set_fade_with_time(LEDC_MODE, X_IN1_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
                ledc_fade_start(LEDC_MODE, X_IN1_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
                
                x_state = MOTOR_FORWARD;
            }
        } else if (right && !left) {
            if (x_state != MOTOR_BACKWARD) {
                ESP_LOGI("motor", "X RIGHT: %.2f°", turret_pos.x_angle);

                ledc_set_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL, 0);

                ledc_update_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL);

                ledc_set_fade_with_time(LEDC_MODE, X_IN2_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
                ledc_fade_start(LEDC_MODE, X_IN2_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

                x_state = MOTOR_BACKWARD;
            }
        } else {
            if (x_state == MOTOR_FORWARD || x_state == MOTOR_BACKWARD) {
                ledc_set_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL, 0);
                ledc_set_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL);
                ledc_update_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL);

                x_state = MOTOR_BRAKING;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
} 

void process_input(char *input) {
    uint16_t len = strcspn(input, "\r\n");
    char *parsed_input = (char *)malloc(len + 1);
    
    memcpy(parsed_input, input, len);
    parsed_input[len] = '\0';

    char *ptr = parsed_input;
    
    if (*ptr == 'A') {
        controls.up_pressed = true;
    } else if (*ptr == 0x30) {
        controls.up_pressed = false;
    }

    if (*(ptr + 1) == 'B') {
        controls.down_pressed = true;
    } else if (*(ptr + 1) == 0x30){
        controls.down_pressed = false;
    }

    if (*(ptr + 2) == 'D') {
        controls.left_pressed = true;
    } else if (*(ptr + 2) == 0x30){
        controls.left_pressed = false;
    }

    if (*(ptr + 3) == 'C') {
        controls.right_pressed = true;
    } else if (*(ptr + 3) == 0x30) {
        controls.right_pressed = false;
    }

    if (*ptr == 0x39 ) {
        DUTY = MAX_PWM_DUTY;
        ESP_LOGI("pars", "Duty set to MAX DUTY: %d", DUTY);
    }

    if (*ptr == 0x32) {
        DUTY = MIN_PWM_DUTY;
        ESP_LOGI("pars", "Duty set to MIN DUTY: %d", DUTY);
    }

    if (*ptr == '+') {
        uint16_t temp_duty = DUTY + DUTY_STEP;

        if (temp_duty > MAX_PWM_DUTY) {
            DUTY = MAX_PWM_DUTY;
        } else {
            DUTY = temp_duty;
        }

        ESP_LOGI("duty", "Duty set to: %d", (int)DUTY);
    }

    if (*ptr == '-') {
        uint16_t temp_duty = DUTY - DUTY_STEP;

        if (temp_duty < MIN_PWM_DUTY) {
            DUTY = MIN_PWM_DUTY;
        } else {
            DUTY = temp_duty;
        }

        ESP_LOGI("duty", "Duty set to: %d", (int)DUTY);
    }

    if (*ptr == 0x20) {
        ESP_LOGI("process input", "Shoot fired!");
        shoot();
    }
    
    if (*ptr == 'Z') {
        ESP_LOGI("input", "Calibrating both axes...");
        if (calibrate_turret() == ESP_OK) {
            ESP_LOGI("input", "Dual calibration successful");
        } else {
            ESP_LOGI("input", "Calibration failed");
        }
    }
    
    // ESP_LOGI("debug", "%s %d", parsed_input, strlen(parsed_input));

    free(parsed_input);
}

void uart_task(void *pvParameters) {
    const char* welcome_msg = "\r\n=== ESP32 PWM Controller ===\r\n> ";

    uart_write_bytes(UART_NUM, welcome_msg, strlen(welcome_msg));

    uint8_t data[UART_BUFF_SIZE];

    while (1) {
        size_t len = uart_read_bytes(UART_NUM, data, UART_BUFF_SIZE - 1, pdMS_TO_TICKS(10));
        data[len] = '\0';

        if (len) {
            process_input((char *)data);
        }
    }
}

void tcp_server_task(void *pvParameters) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (!wifi_ap_started) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("TCP", "Waiting for Access Point to start...");
    }
    
    tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_socket < 0) {
        ESP_LOGE("TCP", "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(tcp_server_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(tcp_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("TCP", "Failed to bind socket");
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    if (listen(tcp_server_socket, 4) < 0) {
        ESP_LOGE("TCP", "Failed to listen");
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI("TCP", "TCP Server listening on 192.168.4.1:%d", TCP_PORT);
    ESP_LOGI("TCP", "Ready to accept connections from Python client");
    
    while (1) {
        client_socket = accept(tcp_server_socket, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket >= 0) {
            ESP_LOGI("TCP", "Python client connected");

            const char* welcome = "ESP32 Turret Controller Ready!\n";
            send(client_socket, welcome, strlen(welcome), 0);
            
            uint8_t buffer[128];
            char line_buffer[32];
            int line_pos = 0;
            
            while (1) {
                int len = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
                if (len <= 0) {
                    ESP_LOGI("TCP", "Client disconnected");
                    break;
                }

                for (int i = 0; i < len; i++) {
                    char c = buffer[i];

                    if (c == '\n' || c == '\r') {
                        if (line_pos > 0) {
                            line_buffer[line_pos] = '\0';
                            ESP_LOGI("TCP", "Cmd: %s", line_buffer);
                            process_input(line_buffer);
                            
                            char response[64];
                            snprintf(response, sizeof(response), "OK: X=%.1f Y=%.1f\n", 
                                    turret_pos.x_angle, turret_pos.y_angle);
                            send(client_socket, response, strlen(response), 0);
                            
                            line_pos = 0; 
                        }
                    }

                    else if (line_pos < sizeof(line_buffer) - 1) {
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
void app_main(void) {
    ESP_LOGI("MAIN", "Starting ESP32 Turret Controller in Access Point mode...");
    
    gpio_reset_pin(TRIGGER_PIN);

    configure_pwm();
    configure_uart();
    init_shot_timer();
    configure_trigger();

    turret_pos.x_angle = 0.0f;
    turret_pos.y_angle = 0.0f;
    turret_pos.target_x = 0.0f;
    turret_pos.target_y = 0.0f;

    // Ініціалізація WiFi Access Point
    ESP_LOGI("MAIN", "Initializing WiFi Access Point...");
    wifi_init_ap();

    ESP_LOGI("MAIN", "Starting tasks...");

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor_control", 8192, NULL, 10, NULL);
    // xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL);
    
    ESP_LOGI("MAIN", "=== ESP32 Turret Controller Ready ===");
    ESP_LOGI("MAIN", "1. Connect to WiFi: %s", WIFI_AP_SSID);
    ESP_LOGI("MAIN", "2. Password: %s", WIFI_AP_PASS);
    ESP_LOGI("MAIN", "3. Run Python script with IP: 192.168.4.1");
    ESP_LOGI("MAIN", "4. Port: %d", TCP_PORT);
}