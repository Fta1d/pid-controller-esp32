#include "trigger.h"
#include "motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "TRIGGER";
static volatile uint32_t last_interrupt_time = 0;
static const uint32_t DEBOUNCE_TIME_MS = 50;

void IRAM_ATTR trigger_interrupt_handler(void *arg) {
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    if (current_time - last_interrupt_time < DEBOUNCE_TIME_MS) {
        return; 
    }
    
    last_interrupt_time = current_time;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(motor_control_event_group, MOTOR_SHOOT_EVENT, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool IRAM_ATTR trigger_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gpio_set_level(TRIGGER_PIN, 0);
    gptimer_stop(timer);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(motor_control_event_group, TRIGGER_RESTORE_EVENT, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return false; 
}

esp_err_t trigger_init_timer(void) {
    ESP_LOGI(TAG, "Initializing shoot timer...");
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz = 1μs resolution
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &shot_timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = trigger_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(shot_timer, &cbs, NULL));
    
    ESP_ERROR_CHECK(gptimer_enable(shot_timer));
    
    ESP_LOGI(TAG, "Shoot timer initialized successfully");
    return ESP_OK;
}

esp_err_t trigger_init(void) {
    ESP_LOGI(TAG, "Initializing trigger system...");

    gpio_config_t gpio_conf[] = {
        {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pin_bit_mask = 1ULL << TRIGGER_PIN
        },
        {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pin_bit_mask = 1ULL << INPUT_TRIGGER_PIN
        }
    };

    for (size_t i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(gpio_config(&gpio_conf[i]));
    }

    ESP_ERROR_CHECK(gpio_set_level(TRIGGER_PIN, 0));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INPUT_TRIGGER_PIN, trigger_interrupt_handler, NULL));
    
    ESP_LOGI(TAG, "Trigger system initialized successfully");
    ESP_LOGI(TAG, "Trigger output pin: %d", TRIGGER_PIN);
    ESP_LOGI(TAG, "Trigger input pin: %d", INPUT_TRIGGER_PIN);
    
    return ESP_OK;
}

void trigger_shoot(void) {
    // ESP_LOGI(TAG, "=== SHOOTING SEQUENCE STARTED ===");

    x_motor.duty_saved = x_motor.duty;
    y_motor.duty_saved = y_motor.duty;

    x_motor.dir_saved = x_motor.dir;
    y_motor.dir_saved = y_motor.dir;

    // ESP_LOGI(TAG, "Motor states saved - X: speed=%.3f dir=%d, Y: speed=%.3f dir=%d", 
    //          analog_state.x_speed_saved, (int)analog_state.x_dir_saved,
    //          analog_state.y_speed_saved, (int)analog_state.y_dir_saved);
    
    motor_stop_all();

    ESP_LOGI(TAG, "Activating trigger for %d ms", shoot_time);
    gpio_set_level(TRIGGER_PIN, 1);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = MS_TO_US(shoot_time), 
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false, 
    };

    gptimer_set_alarm_action(shot_timer, &alarm_config);
    gptimer_set_raw_count(shot_timer, 0);
    gptimer_start(shot_timer);
    
    ESP_LOGI(TAG, "Shoot sequence initiated, timer started");
}