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
#include "pid.h"
#include "backlash_compensator.h"
#include "accel.h"
#include "utils.h"

uint16_t shoot_time = DEF_SHOOT_TIME;
gptimer_handle_t shot_timer = NULL;

EventGroupHandle_t motor_control_event_group;

motor_t x_motor = {0};
motor_t y_motor = {0};
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
    ESP_ERROR_CHECK(uart_init());
    ESP_ERROR_CHECK(trigger_init());
    ESP_ERROR_CHECK(trigger_init_timer());
    ESP_ERROR_CHECK(lsm303_init());
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    ESP_ERROR_CHECK(wifi_init_ap());

    ESP_LOGI(TAG, "Starting tasks...");

    motor_control_event_group = xEventGroupCreate();
    
    uart_task_create();
    tcp_server_task_create();
    motor_task_create();
    pid_task_create();
    encoder_task_create();
    backlash_compensator_task_create();

    ESP_LOGI(TAG, "=== ESP32 Turret Controller Ready ===");
    ESP_LOGI(TAG, "1. Connect to WiFi: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "2. Password: %s", WIFI_AP_PASS);
    ESP_LOGI(TAG, "3. Run Python script with IP: 192.168.4.1");
    ESP_LOGI(TAG, "4. Port: %d", TCP_PORT);

    scan_i2c_bus(Y_ENCODER_I2C_PORT);
}