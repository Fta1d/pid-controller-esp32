#include "backlash_compensator.h"
#include "motor.h"
#include "accel.h"

#define MOTION_THRESHOLD            200

static const char *TAG = "BACKLASH_COMP";
static backlash_compensator_t backlash_comp_handle = {0};

void skip_backlash(bool dir, uint32_t target_speed) {
    if (backlash_comp_handle.backlash_in_ticks == 0) {
        ESP_LOGE(TAG, "Backlash compensator not calibrated");
        return;
    }
    
    ESP_LOGI(TAG, "Applying backlash compensation");
    
    backlash_comp_handle.compensation_in_progress = true;

    x_motor.duty_saved = target_speed;
    x_motor.dir_saved = dir;

    motor_set_speed_analog('X', MAX_PWM_DUTY, dir);
    
    ESP_LOGI(TAG, "Compensating at max speed for %d ticks", 
             (int)backlash_comp_handle.backlash_in_ticks);
    
    // Чекаємо компенсацію
    vTaskDelay(backlash_comp_handle.backlash_in_ticks);
    
    // Відновлюємо цільову швидкість
    motor_set_speed_analog('X', target_speed, dir);
    
    backlash_comp_handle.compensation_in_progress = false;
    
    ESP_LOGI(TAG, "Backlash compensation completed");
}

// void skip_backlash(bool dir, uint32_t target_speed) {
//     if (backlash_comp_handle.backlash_in_ticks == 0) {
//         ESP_LOGE(TAG, "Backlash compensator not calibrated");
//         return;
//     }
    
//     ESP_LOGI(TAG, "Applying backlash compensation");

//     x_motor.duty_saved = target_speed;
//     x_motor.dir_saved = dir;

//     xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);

//     x_motor.duty = MAX_PWM_DUTY;
//     x_motor.dir = dir;
//     xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
//     vTaskDelay(pdMS_TO_TICKS(5));
//     ESP_LOGI(TAG, "Curr duty %lu %lu", ledc_get_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL), ledc_get_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL));
    
//     vTaskDelay(backlash_comp_handle.backlash_in_ticks);
    
//     // xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);
//     // vTaskDelay(pdMS_TO_TICKS(5));

//     motor_restore_states();
    
//     ESP_LOGI(TAG, "Backlash compensation completed in %d ticks", 
//              (int)backlash_comp_handle.backlash_in_ticks);
// }

void calculate_backlash(/**/) {
    // if (backlash_handle == NULL) {
    //     ESP_LOGE(TAG, "Backlash handle is NULL!");
    //     return;
    // }

    float ax_start, ay_start, az_start, ax_curr, ay_curr, az_curr;
    lsm303_read_accel_filtered(&ax_start, &ay_start, &az_start); 

    
    x_motor.duty = MAX_PWM_DUTY;
    x_motor.dir = 1;
    xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
    vTaskDelay(pdMS_TO_TICKS(300));
    xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);
    vTaskDelay(pdMS_TO_TICKS(100));

    x_motor.duty = MAX_PWM_DUTY;
    x_motor.dir = 0;
    xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);

    TickType_t end_ticks, start_ticks = xTaskGetTickCount();
    int timeout_counter = 0;
    int16_t x_curr, y_curr, z_curr;

    do {
        // lsm303_read_accel_filtered(&ax_curr, &ay_curr, &az_curr);
        lsm303_read_accel(&x_curr, &y_curr, &z_curr);
        vTaskDelay(pdMS_TO_TICKS(5));

        ESP_LOGI(TAG, "X %d", (int)x_curr);

        if (++timeout_counter >= 1000) {
            ESP_LOGE(TAG, "Timeout!");
            break;
        }
    } while (x_curr < MOTION_THRESHOLD);
    
    xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);

    end_ticks = xTaskGetTickCount();

    backlash_comp_handle.backlash_in_ticks = end_ticks - start_ticks;
    ESP_LOGW(TAG, "Backlash in tiks: %d", (int)backlash_comp_handle.backlash_in_ticks);
}

void backlash_compensator_task(void *pvParameters) {
    int16_t x_raw, y_raw, z_raw;
    float ax_filtered, ay_filtered, az_filtered;
    
    while (true) {
        if (lsm303_read_accel_filtered(&ax_filtered, &ay_filtered, &az_filtered) == ESP_OK) {
            // ESP_LOGI(TAG, "Filtered: X=%.1f Y=%.1f Z=%.1f", ax_filtered, ay_filtered, az_filtered);
        }
        if (lsm303_read_accel(&x_raw, &y_raw, &z_raw) == ESP_OK) {
            // ESP_LOGI(TAG, "Raw: x=%d", (int)x_raw/*, (int)y_raw, (int)z_raw*/);
        }
        

        vTaskDelay(pdMS_TO_TICKS(50));  
    }
    
}

void backlash_compensator_task_create() {
    xTaskCreate(backlash_compensator_task, "backlash_compensator", 4096, NULL, 3, NULL);
}

backlash_compensator_t *backlash_compensator_init() {
    backlash_compensator_t *backlash_handle = calloc(1, sizeof(backlash_compensator_t));
    
    if (backlash_handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for backlash compensator");
        return NULL;
    }

    backlash_handle->backlash_in_angle = 0.0f;
    backlash_handle->backlash_in_ticks = 0;
    backlash_handle->ena = false;
    
    ESP_LOGI(TAG, "Backlash compensator initialized");

    return backlash_handle;
}

void backlash_compensator_deinit(backlash_compensator_t *handle) {
    if (handle != NULL) {
        free(handle);
    }
}

void backlash_comp_enable(bool val) {
    backlash_comp_handle.ena = val;
}

bool backlash_compensator_is_enabled() {
    return backlash_comp_handle.ena;
}

bool backlash_compensation_in_progress(void) {
    return backlash_comp_handle.compensation_in_progress;
}