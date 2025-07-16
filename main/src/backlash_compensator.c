#include "backlash_compensator.h"
#include "accel.h"

#define MOTION_THRESHOLD            200

static const char *TAG = "BACKLASH_COMP";
static backlash_compensator_t backlash_compensator_handle = {0};

float read_gyro() {
    static int calls = 0;

    if (calls++ > 50) {
        return 1.1;
    } else {
        return 10.0;
    }
}

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

    backlash_compensator_handle.backlash_in_ticks = end_ticks - start_ticks;
    ESP_LOGW(TAG, "Backlash in tiks: %d", (int)backlash_compensator_handle.backlash_in_ticks);
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