#include "backlash_compensator.h"

#define LSM303DLHC_WHO_AM_I_REG     0x0F
#define LSM303DLHC_ADDR             0x19    // 0011001b = 0x19
#define OUT_X_H_M_REG               0x03    // X axis low byte
#define OUT_Y_H_M_REG               0x07    // Y axis low byte
#define OUT_Z_H_M_REG               0x05    // Z axis low byte

static const char *TAG = "LSM303_ACCEL";
static i2c_port_t i2c_port = I2C_NUM_0;

void lsm303_set_i2c_port(i2c_port_t port) {
    i2c_port = port;
}

esp_err_t lsm303_write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM303DLHC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t lsm303_read_reg(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM303DLHC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM303DLHC_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t lsm303_read_mag(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM303DLHC_ADDR << 1) | 0, true);  // Write
    i2c_master_write_byte(cmd, OUT_X_H_M_REG | 0x80, true);  // 0x80 = auto-increment
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM303DLHC_ADDR << 1) | 1, true);  // Read
    i2c_master_read(cmd, data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_port, cmd, 100);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *x = (int16_t)((data[1] << 8) | data[0]);
        *y = (int16_t)((data[3] << 8) | data[2]);
        *z = (int16_t)((data[5] << 8) | data[4]);
    }
    
    return ret;
}

esp_err_t lsm303_mag_init(i2c_port_t port) {
    i2c_port = port;
    
    ESP_LOGI(TAG, "Initializing LSM303DLHC magnetometer on port %d", port);

    uint8_t who_am_i;
    esp_err_t ret = lsm303_read_reg(LSM303DLHC_WHO_AM_I_REG, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
    if (who_am_i != 0x33) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I (expected 0x33), but continuing...");
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "LSM303DLHC magnetometer initialized successfully");
    
    return ESP_OK;
}

float read_gyro() {
    static int calls = 0;

    if (calls++ > 50) {
        return 1.1;
    } else {
        return 10.0;
    }
}

void skip_backlash() {

}

void calculate_backlash(backlash_compensator_t *backlash_handle) {
    if (backlash_handle == NULL) {
        ESP_LOGE(TAG, "Backlash handle is NULL!");
        return;
    }

    TickType_t end_ticks, start_ticks = xTaskGetTickCount();
    float start_gyro = 1.0; // read_gyroscope();

    x_motor.duty = MAX_PWM_DUTY;
    x_motor.dir = 1;

    xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
    vTaskDelay(pdMS_TO_TICKS(100));
    xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);

    x_motor.duty = MAX_PWM_DUTY;
    x_motor.dir = 0;

    xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);

    while (abs(read_gyro() - start_gyro) < GYRO_THRESHOLD) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);

    end_ticks = xTaskGetTickCount();

    backlash_handle->backlash_in_ticks = end_ticks - start_ticks;

}

void init_acel() {
    // int16_t x, y, z;

    // while (1) {
    //     // read_accel(&x, &y, &z);
    //     ESP_LOGI(TAG, "X %d Y %d Z %d", (int)x, (int)y, (int)z);

    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
    

    
}

void backlash_compensator_task(void *pvParameters) {
    int16_t x_raw, y_raw, z_raw;
    
    while (true) {
        lsm303_read_mag(&x_raw, &y_raw, &z_raw);

        ESP_LOGI(TAG, "X ms: %d Y ms: %d Z ms: %d", (int)x_raw, (int)y_raw, (int)z_raw);

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