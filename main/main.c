#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "config.h"
#include "motor.h"
#include "wifi_ap.h"
#include "tcp_server.h"
#include "trigger.h"
#include "encoder.h"
#include "uart.h"

uint16_t DUTY = DEFAULT_DUTY;
uint16_t shoot_time = DEF_SHOOT_TIME;
gptimer_handle_t shot_timer = NULL;

motor_t motor_x = {
    .in1_channel = X_IN1_LEDC_CHANNEL,
    .in2_channel = X_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .name = "X"
};

motor_t motor_y = {
    .in1_channel = Y_IN1_LEDC_CHANNEL,
    .in2_channel = Y_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .name = "Y"
};

EventGroupHandle_t motor_control_event_group;
TaskHandle_t motor_control_task_handle;

turret_position_t turret_pos = {0};
analog_motor_state_t analog_state = {0};
motor_state_t y_state = MOTOR_STOPPED;
motor_state_t x_state = MOTOR_STOPPED;

static const char *TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP32 Turret Controller in Access Point mode...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing subsystems...");
    
    ESP_ERROR_CHECK(motor_init_pwm());
    ESP_ERROR_CHECK(encoder_init_dual());
    // ESP_ERROR_CHECK(uart_init());
    ESP_ERROR_CHECK(trigger_init());
    ESP_ERROR_CHECK(trigger_init_timer());

    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    ESP_ERROR_CHECK(wifi_init_ap());

    ESP_LOGI(TAG, "Starting tasks...");

    motor_control_event_group = xEventGroupCreate();
    
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 15, NULL);
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 10, &motor_control_task_handle);
    trigger_set_motor_task_handle(motor_control_task_handle);
    // xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL); 
    
    ESP_LOGI(TAG, "=== ESP32 Turret Controller Ready ===");
    ESP_LOGI(TAG, "1. Connect to WiFi: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "2. Password: %s", WIFI_AP_PASS);
    ESP_LOGI(TAG, "3. Run Python script with IP: 192.168.4.1");
    ESP_LOGI(TAG, "4. Port: %d", TCP_PORT);
}