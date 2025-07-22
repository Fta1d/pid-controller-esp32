#include "accel.h"

static const char *TAG = "ACCEL";
static i2c_port_t default_i2c_port = Y_ENCODER_I2C_PORT;

static accel_filter_t accel_filter = {0};

// ================ I2C WRITE/READ FUNCTIONS ================

esp_err_t i2c_write_byte_port(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: device=0x%02X, reg=0x%02X, data=0x%02X, error=%s", 
                 device_addr, reg_addr, data, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    return i2c_write_byte_port(default_i2c_port, device_addr, reg_addr, data);
}

esp_err_t i2c_read_byte_port(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    // Запис адреси регістру
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register address failed: device=0x%02X, reg=0x%02X, error=%s", 
                 device_addr, reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read data failed: device=0x%02X, reg=0x%02X, error=%s", 
                 device_addr, reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_read_byte_port(default_i2c_port, device_addr, reg_addr, data);
}

esp_err_t i2c_read_bytes_port(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_ERR_INVALID_ARG;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register address failed: device=0x%02X, reg=0x%02X, error=%s", 
                 device_addr, reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read data failed: device=0x%02X, reg=0x%02X, len=%d, error=%s", 
                 device_addr, reg_addr, len, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_read_bytes_port(default_i2c_port, device_addr, reg_addr, data, len);
}

esp_err_t i2c_read_bytes_autoincrement_port(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    uint8_t reg_with_autoincrement = reg_addr | 0x80;
    return i2c_read_bytes_port(port, device_addr, reg_with_autoincrement, data, len);
}

esp_err_t i2c_read_bytes_autoincrement(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_read_bytes_autoincrement_port(default_i2c_port, device_addr, reg_addr, data, len);
}

esp_err_t i2c_write_bytes_port(i2c_port_t port, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_ERR_INVALID_ARG;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write bytes failed: device=0x%02X, reg=0x%02X, len=%d, error=%s", 
                 device_addr, reg_addr, len, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_write_bytes_port(default_i2c_port, device_addr, reg_addr, data, len);
}

bool i2c_device_exists_port(i2c_port_t port, uint8_t device_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

bool i2c_device_exists(uint8_t device_addr) {
    return i2c_device_exists_port(default_i2c_port, device_addr);
}

esp_err_t lsm303_accel_enable_hpf(uint8_t cutoff_freq) {
    uint8_t hpf_config = 0x08 | cutoff_freq;  // HPM = 01 (normal mode), HPCF = cutoff_freq
    
    esp_err_t ret = i2c_write_byte(LSM303DLHC_ACCEL_ADDR, 0x21, hpf_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable HPF");
        return ret;
    }
    
    ESP_LOGI(TAG, "High-Pass Filter enabled with cutoff frequency %d", cutoff_freq);
    return ESP_OK;
}

// ================ SPECIAL ACCEL FUNCTIONS ================

esp_err_t lsm303_accel_init() {
    ESP_LOGI(TAG, "Initializing LSM303DLHC accelerometer");
    
    if (!i2c_device_exists(LSM303DLHC_ACCEL_ADDR)) {
        ESP_LOGE(TAG, "Accelerometer not found at address 0x%02X", LSM303DLHC_ACCEL_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    
    uint8_t who_am_i;
    esp_err_t ret = i2c_read_byte(LSM303DLHC_ACCEL_ADDR, 0x0F, &who_am_i);
    if (ret != ESP_OK || who_am_i != 0x33) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x33)", who_am_i);
    }
    
    //1.6 kHz, all axis, normal mode
    ret = i2c_write_byte(LSM303DLHC_ACCEL_ADDR, 0x20, 0x97);
    if (ret != ESP_OK) return ret;

    // High pass filter
    ret = lsm303_accel_enable_hpf(0x03);  
    if (ret != ESP_OK) return ret;
    
    // ±2g, high resolution
    ret = i2c_write_byte(LSM303DLHC_ACCEL_ADDR, 0x23, 0x08);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Accelerometer initialized successfully");
    return ESP_OK;
}

esp_err_t lsm303_mag_init() {
    ESP_LOGI(TAG, "Initializing LSM303DLHC magnetometer");
    
    if (!i2c_device_exists(LSM303DLHC_MAG_ADDR)) {
        ESP_LOGE(TAG, "Magnetometer not found at address 0x%02X", LSM303DLHC_MAG_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    
    // 220 Hz
    esp_err_t ret = i2c_write_byte(LSM303DLHC_MAG_ADDR, 0x00, 0x1C);
    if (ret != ESP_OK) return ret;
    
    // ±1.3 Gauss
    ret = i2c_write_byte(LSM303DLHC_MAG_ADDR, 0x01, 0x20);
    if (ret != ESP_OK) return ret;
    
    // Continuous mode
    ret = i2c_write_byte(LSM303DLHC_MAG_ADDR, 0x02, 0x00);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Magnetometer initialized successfully");
    return ESP_OK;
}

esp_err_t lsm303_init() {
    ESP_LOGI(TAG, "Initializing LSM303DLHC module");
    
    esp_err_t ret1 = lsm303_accel_init();
    esp_err_t ret2 = lsm303_mag_init();
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "LSM303DLHC module initialized successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "LSM303DLHC initialization failed");
        return ESP_FAIL;
    }
}

esp_err_t lsm303_read_accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    
    esp_err_t ret = i2c_read_bytes_autoincrement(LSM303DLHC_ACCEL_ADDR, 0x28, data, 6);
    if (ret == ESP_OK) {
        // Little Endian
        *x = (int16_t)((data[1] << 8) | data[0]);
        *y = (int16_t)((data[3] << 8) | data[2]);
        *z = (int16_t)((data[5] << 8) | data[4]);
    }
    
    return ret;
}

esp_err_t lsm303_read_mag(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    
    esp_err_t ret = i2c_read_bytes_autoincrement(LSM303DLHC_MAG_ADDR, 0x03, data, 6);
    if (ret == ESP_OK) {
        // Big Endian, X, Z, Y
        *x = (int16_t)((data[0] << 8) | data[1]);
        *z = (int16_t)((data[2] << 8) | data[3]);
        *y = (int16_t)((data[4] << 8) | data[5]);
    }
    
    return ret;
}

float moving_average(float *buffer, float new_value, int *index, bool *full) {
    buffer[*index] = new_value;
    *index = (*index + 1) % FILTER_WINDOW_SIZE;
    
    if (*index == 0) *full = true;
    
    float sum = 0;
    int count = *full ? FILTER_WINDOW_SIZE : *index;
    
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    
    return sum / count;
}

esp_err_t lsm303_read_accel_filtered(float *ax, float *ay, float *az) {
    int16_t x_raw, y_raw, z_raw;
    esp_err_t ret = lsm303_read_accel(&x_raw, &y_raw, &z_raw);
    if (ret != ESP_OK) return ret;
    
    float ax_raw = (float)x_raw;
    float ay_raw = (float)y_raw;
    float az_raw = (float)z_raw;
    
    float ax_avg = moving_average(accel_filter.ax_buffer, ax_raw, 
                                  &accel_filter.buffer_index, &accel_filter.buffer_full);
    float ay_avg = moving_average(accel_filter.ay_buffer, ay_raw, 
                                  &accel_filter.buffer_index, &accel_filter.buffer_full);
    float az_avg = moving_average(accel_filter.az_buffer, az_raw, 
                                  &accel_filter.buffer_index, &accel_filter.buffer_full);
    
    if (!accel_filter.initialized) {
        accel_filter.ax_filtered = ax_avg;
        accel_filter.ay_filtered = ay_avg;
        accel_filter.az_filtered = az_avg;
        accel_filter.initialized = true;
    } else {
        const float alpha = 0.5f; 
        accel_filter.ax_filtered = alpha * accel_filter.ax_filtered + (1 - alpha) * ax_avg;
        accel_filter.ay_filtered = alpha * accel_filter.ay_filtered + (1 - alpha) * ay_avg;
        accel_filter.az_filtered = alpha * accel_filter.az_filtered + (1 - alpha) * az_avg;
    }
    
    *ax = accel_filter.ax_filtered;
    *ay = accel_filter.ay_filtered;
    *az = accel_filter.az_filtered;
    
    return ESP_OK;
}

