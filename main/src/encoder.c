#include "esp_log.h"

#include "encoder.h"
#include "config.h"
#include "motor.h"

static const char *TAG = "ENCODER";

as5600_handle_t x_encoder = NULL;
as5600_handle_t y_encoder = NULL;
as5600_sliding_window_t *x_sw = NULL;
as5600_sliding_window_t *y_sw = NULL;

esp_err_t encoder_init_dual(void) {
    ESP_LOGI(TAG, "Initializing dual AS5600 encoders...");
    
    // ========== X ENCODER (I2C1) ==========
    // 
    // as5600_config_t x_config = {
    //     .i2c_port = X_ENCODER_I2C_PORT,
    //     .scl_pin = X_ENCODER_SCL_PIN,
    //     .sda_pin = X_ENCODER_SDA_PIN,
    //     .clk_speed = I2C_FREQ_HZ,
    //     .enable_pullups = true
    // };
    
    // x_encoder = as5600_init(&x_config);
    // if (x_encoder == NULL) {
    //     ESP_LOGE(TAG, "Failed to initialize X encoder (I2C1)");
    //     return ESP_FAIL;
    // }
    
    // ESP_LOGI(TAG, "X encoder initialized on I2C1 (SCL=%d, SDA=%d)", 
    //          X_ENCODER_SCL_PIN, X_ENCODER_SDA_PIN);

    // if (as5600_is_connected(x_encoder) == ESP_OK) {
    //     ESP_LOGI(TAG, "X encoder connection verified");
    // } else {
    //     ESP_LOGW(TAG, "X encoder connection check failed");
    // }
    
    // ========== Y ENCODER (I2C0) ==========
    as5600_config_t y_config = {
        .i2c_port = Y_ENCODER_I2C_PORT,
        .scl_pin = Y_ENCODER_SCL_PIN,
        .sda_pin = Y_ENCODER_SDA_PIN,
        .clk_speed = I2C_FREQ_HZ,
        .enable_pullups = true
    };
    
    y_encoder = as5600_init(&y_config);
    if (y_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to initialize Y encoder (I2C0)");
        if (x_encoder != NULL) {
            as5600_deinit(x_encoder);
            x_encoder = NULL;
        }
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Y encoder initialized on I2C0 (SCL=%d, SDA=%d)", 
             Y_ENCODER_SCL_PIN, Y_ENCODER_SDA_PIN);
    
    if (as5600_is_connected(y_encoder) == ESP_OK) {
        ESP_LOGI(TAG, "Y encoder connection verified");
    } else {
        ESP_LOGW(TAG, "Y encoder connection check failed");
    }

    if (x_encoder != NULL) {
        as5600_print_diagnostics(x_encoder);
    }
    
    as5600_print_diagnostics(y_encoder);

    // as5600_read_angle_degrees(x_encoder, &x_motor.angle);
    as5600_read_angle_degrees(y_encoder, &y_motor.angle);

    // Initialize sliding windows
    // x_sw = as5600_sliding_window_create(10);
    y_sw = as5600_sliding_window_create(10);
    
    ESP_LOGI(TAG, "Encoder initialization completed");
    return ESP_OK;
}

esp_err_t encoder_read_y_angle(void) {
    if (y_encoder == NULL) {
        ESP_LOGE(TAG, "Y encoder not initialized");
        return ESP_FAIL;
    }
    
    return as5600_read_angle_degrees_sliding(y_encoder, y_sw, &y_motor.angle);
}

esp_err_t encoder_read_x_angle(void) {
    if (x_encoder == NULL) {
        ESP_LOGE(TAG, "X encoder not initialized");
        return ESP_FAIL;
    }
    
    return as5600_read_angle_degrees_sliding(x_encoder, x_sw, &x_motor.angle);
}

esp_err_t encoder_calibrate_turret(void) {
    ESP_LOGI(TAG, "Starting dual encoder calibration...");
    
    esp_err_t x_ret = ESP_OK;
    esp_err_t y_ret = ESP_FAIL;

    if (x_encoder != NULL) {
        x_ret = as5600_set_current_as_zero(x_encoder);
        if (x_ret == ESP_OK) {
            ESP_LOGI(TAG, "X encoder calibrated successfully");
        } else {
            ESP_LOGE(TAG, "X encoder calibration failed: %s", esp_err_to_name(x_ret));
        }
    } else {
        ESP_LOGW(TAG, "X encoder not initialized, skipping calibration");
    }
    
    if (y_encoder != NULL) {
        y_ret = as5600_set_current_as_zero(y_encoder);
        if (y_ret == ESP_OK) {
            ESP_LOGI(TAG, "Y encoder calibrated successfully");
        } else {
            ESP_LOGE(TAG, "Y encoder calibration failed: %s", esp_err_to_name(y_ret));
        }
    } else {
        ESP_LOGE(TAG, "Y encoder not initialized");
        return ESP_FAIL;
    }
    
    if (y_ret == ESP_OK && (x_encoder == NULL || x_ret == ESP_OK)) {
        x_motor.angle = 0.0f;
        y_motor.angle = 0.0f;
        ESP_LOGI(TAG, "Dual calibration complete - both axes zeroed");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Calibration failed - X: %s, Y: %s", 
                esp_err_to_name(x_ret), esp_err_to_name(y_ret));
        return ESP_FAIL;
    }
}

void encoder_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing encoders...");
    
    if (x_encoder != NULL) {
        as5600_deinit(x_encoder);
        x_encoder = NULL;
        ESP_LOGI(TAG, "X encoder deinitialized");
    }
    
    if (y_encoder != NULL) {
        as5600_deinit(y_encoder);
        y_encoder = NULL;
        ESP_LOGI(TAG, "Y encoder deinitialized");
    }
}

esp_err_t encoder_get_status(void) {
    if (y_encoder == NULL) {
        return ESP_FAIL;
    }
    
    if (x_encoder != NULL) {
        if (as5600_is_connected(x_encoder) != ESP_OK) {
            ESP_LOGW(TAG, "X encoder connection lost");
            return ESP_FAIL;
        }
    }
    
    if (as5600_is_connected(y_encoder) != ESP_OK) {
        ESP_LOGW(TAG, "Y encoder connection lost");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static void encoder_task(void *pvParemeters) {
    TickType_t encoder_task_freq = pdMS_TO_TICKS(20);

    while (true) {
        TickType_t last_wake_time = xTaskGetTickCount();

        encoder_read_y_angle();
        // ESP_LOGI(TAG, "ANGLE Y: %f", y_motor.angle);

        // if (LOG_STATE) printf("%f || %f\n", motor_x.angle, motor_y.angle);
    
        // if (x_motor.duty > 0 && x_state != MOTOR_STOPPED ) {
        //     if ((x_motor.dir && x_motor.angle >= MAX_X_ANGLE) ||
        //         (!x_motor.dir && x_motor.angle <= MIN_X_ANGLE)) {
        //             motor_stop(&motor_x_channels);
        //             ESP_LOGE(TAG, "Emergency stop X at %.2f degrees", x_motor.angle);
        //     }
        // }
        
        if (y_motor.duty > 0 && y_state != MOTOR_STOPPED) {
            if ((!y_motor.dir && y_motor.angle >= MAX_Y_ANGLE) ||
                (y_motor.dir && y_motor.angle <= MIN_Y_ANGLE)) {
                    motor_stop(&motor_y_channels);
                    ESP_LOGE(TAG, "Emergency stop Y at %.2f degrees", y_motor.angle);
            }
        }

        vTaskDelayUntil(&last_wake_time, encoder_task_freq);
    }
    
}

void encoder_task_create(void) {
    xTaskCreate(encoder_task, "encoder", 4096, NULL, 9, NULL);
}