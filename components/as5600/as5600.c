/**
 * @file as5600.c
 * @brief AS5600 Magnetic Rotary Position Sensor Driver Implementation
 */

#include "as5600.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AS5600";

/**
 * @brief AS5600 device structure
 */
struct as5600_dev_t {
    i2c_port_t i2c_port;
    as5600_config_t config;
    bool initialized;
};

// ====================================================================================
// PRIVATE FUNCTIONS
// ====================================================================================

/**
 * @brief Write 8-bit register
 */
static esp_err_t as5600_write_reg_8(as5600_handle_t handle, uint8_t reg_addr, uint8_t data)
{   
    esp_err_t ret = ESP_OK; 
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(cmd != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create I2C command");
    
    ESP_GOTO_ON_ERROR(i2c_master_start(cmd), cleanup, TAG, "I2C start error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true), cleanup, TAG, "I2C write address error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, reg_addr, true), cleanup, TAG, "I2C write register error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, data, true), cleanup, TAG, "I2C write data error");
    ESP_GOTO_ON_ERROR(i2c_master_stop(cmd), cleanup, TAG, "I2C stop error");
    
    ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(1000));
    
cleanup:
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read 8-bit register
 */
static esp_err_t as5600_read_reg_8(as5600_handle_t handle, uint8_t reg_addr, uint8_t *data)
{
    esp_err_t ret = ESP_OK; 
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Data pointer is NULL");
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_RETURN_ON_FALSE(cmd != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create I2C command");
    
    ESP_GOTO_ON_ERROR(i2c_master_start(cmd), cleanup, TAG, "I2C start error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true), cleanup, TAG, "I2C write address error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, reg_addr, true), cleanup, TAG, "I2C write register error");
    ESP_GOTO_ON_ERROR(i2c_master_start(cmd), cleanup, TAG, "I2C restart error");
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_READ, true), cleanup, TAG, "I2C write read address error");
    ESP_GOTO_ON_ERROR(i2c_master_read_byte(cmd, data, I2C_MASTER_NACK), cleanup, TAG, "I2C read data error");
    ESP_GOTO_ON_ERROR(i2c_master_stop(cmd), cleanup, TAG, "I2C stop error");
    
    ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(1000));
    
cleanup:
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read 16-bit register
 */
static esp_err_t as5600_read_reg_16(as5600_handle_t handle, uint8_t reg_addr, uint16_t *data)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Data pointer is NULL");
    
    uint8_t msb, lsb;
    ESP_RETURN_ON_ERROR(as5600_read_reg_8(handle, reg_addr, &msb), TAG, "Failed to read MSB");
    ESP_RETURN_ON_ERROR(as5600_read_reg_8(handle, reg_addr + 1, &lsb), TAG, "Failed to read LSB");
    
    *data = (msb << 8) | lsb;
    return ESP_OK;
}

/**
 * @brief Write 16-bit register
 */
static esp_err_t as5600_write_reg_16(as5600_handle_t handle, uint8_t reg_addr, uint16_t data)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    ESP_RETURN_ON_ERROR(as5600_write_reg_8(handle, reg_addr, (data >> 8) & 0xFF), TAG, "Failed to write MSB");
    ESP_RETURN_ON_ERROR(as5600_write_reg_8(handle, reg_addr + 1, data & 0xFF), TAG, "Failed to write LSB");
    
    return ESP_OK;
}

/**
 * @brief Initialize I2C master
 */
static esp_err_t as5600_init_i2c(const as5600_config_t *config)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = config->enable_pullups,
        .scl_pullup_en = config->enable_pullups,
        .master.clk_speed = config->clk_speed,
    };
    
    ESP_RETURN_ON_ERROR(i2c_param_config(config->i2c_port, &i2c_conf), TAG, "I2C param config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(config->i2c_port, I2C_MODE_MASTER, 0, 0, 0), TAG, "I2C driver install failed");
    
    return ESP_OK;
}

// ====================================================================================
// PUBLIC FUNCTIONS - INITIALIZATION
// ====================================================================================

as5600_handle_t as5600_init(const as5600_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, NULL, TAG, "Config is NULL");
    
    as5600_handle_t handle = calloc(1, sizeof(struct as5600_dev_t));
    ESP_RETURN_ON_FALSE(handle != NULL, NULL, TAG, "Failed to allocate memory");
    
    // Copy configuration
    memcpy(&handle->config, config, sizeof(as5600_config_t));
    handle->i2c_port = config->i2c_port;
    
    // Initialize I2C
    if (as5600_init_i2c(config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        free(handle);
        return NULL;
    }
    
    // Test connection
    if (as5600_is_connected(handle) != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 not responding");
        i2c_driver_delete(handle->i2c_port);
        free(handle);
        return NULL;
    }
    
    handle->initialized = true;
    ESP_LOGI(TAG, "AS5600 initialized successfully");
    
    return handle;
}

esp_err_t as5600_deinit(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    if (handle->initialized) {
        i2c_driver_delete(handle->i2c_port);
        handle->initialized = false;
    }
    
    free(handle);
    ESP_LOGI(TAG, "AS5600 deinitialized");
    
    return ESP_OK;
}

esp_err_t as5600_is_connected(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    uint8_t test_data;
    return as5600_read_reg_8(handle, AS5600_REG_STATUS, &test_data);
}

// ====================================================================================
// PUBLIC FUNCTIONS - DATA READING
// ====================================================================================

esp_err_t as5600_read_data(as5600_handle_t handle, as5600_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Data pointer is NULL");
    
    memset(data, 0, sizeof(as5600_data_t));
    
    // Read all registers
    ESP_RETURN_ON_ERROR(as5600_read_reg_16(handle, AS5600_REG_RAW_ANGLE_H, &data->raw_angle), TAG, "Failed to read raw angle");
    ESP_RETURN_ON_ERROR(as5600_read_reg_16(handle, AS5600_REG_ANGLE_H, &data->angle), TAG, "Failed to read angle");
    ESP_RETURN_ON_ERROR(as5600_read_reg_8(handle, AS5600_REG_STATUS, &data->status), TAG, "Failed to read status");
    ESP_RETURN_ON_ERROR(as5600_read_reg_8(handle, AS5600_REG_AGC, &data->agc), TAG, "Failed to read AGC");
    ESP_RETURN_ON_ERROR(as5600_read_reg_16(handle, AS5600_REG_MAGNITUDE_H, &data->magnitude), TAG, "Failed to read magnitude");
    
    // Convert to degrees and radians
    data->angle_degrees = as5600_raw_to_degrees(data->angle);
    data->angle_radians = as5600_raw_to_radians(data->angle);
    
    // Parse status flags
    data->magnet_detected = !(data->status & AS5600_STATUS_MD);
    data->magnet_too_strong = (data->status & AS5600_STATUS_ML) != 0;
    data->magnet_too_weak = (data->status & AS5600_STATUS_MD) != 0;
    
    return ESP_OK;
}

esp_err_t as5600_read_raw_angle(as5600_handle_t handle, uint16_t *raw_angle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(raw_angle != NULL, ESP_ERR_INVALID_ARG, TAG, "Raw angle pointer is NULL");
    
    return as5600_read_reg_16(handle, AS5600_REG_RAW_ANGLE_H, raw_angle);
}

esp_err_t as5600_read_angle(as5600_handle_t handle, uint16_t *angle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(angle != NULL, ESP_ERR_INVALID_ARG, TAG, "Angle pointer is NULL");
    
    return as5600_read_reg_16(handle, AS5600_REG_ANGLE_H, angle);
}

esp_err_t as5600_read_angle_degrees(as5600_handle_t handle, float *degrees)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(degrees != NULL, ESP_ERR_INVALID_ARG, TAG, "Degrees pointer is NULL");
    
    uint16_t angle;
    ESP_RETURN_ON_ERROR(as5600_read_angle(handle, &angle), TAG, "Failed to read angle");
    
    *degrees = as5600_raw_to_degrees(angle);
    return ESP_OK;
}

int as5600_read_angle_degrees_sliding(as5600_handle_t handle, as5600_sliding_window_t *sw, float *degrees) {
    if (handle == NULL || sw == NULL || degrees == NULL) {
        printf("%s: Invalid parameters\n", TAG);
        return -1;
    }
    
    uint16_t angle;
    if (as5600_read_angle(handle, &angle) == -1) {
        printf("%s: Failed to read angle\n", TAG);
        return -1;
    }
    
    float new_value = as5600_raw_to_degrees(angle);
    
    if (sw->count >= sw->window_size) {
        sw->last_sum -= sw->samples[sw->index];
    }
    
    sw->samples[sw->index] = new_value;
    sw->last_sum += new_value;
    sw->index = (sw->index + 1) % sw->window_size;
    
    if (sw->count < sw->window_size) {
        sw->count++;
    }
    
    *degrees = sw->last_sum / sw->count;
    return 0;
}

as5600_sliding_window_t* as5600_sliding_window_create(uint16_t window_size) {
    if (window_size == 0) {
        printf("%s: Window size must be > 0\n", TAG);
        return NULL;
    }
    
    as5600_sliding_window_t *sw = malloc(sizeof(as5600_sliding_window_t));
    if (sw == NULL) {
        printf("%s: Failed to allocate sliding window structure\n", TAG);
        return NULL;
    }
    
    sw->samples = calloc(window_size, sizeof(float));
    if (sw->samples == NULL) {
        printf("%s: Failed to allocate buffer\n", TAG);
        free(sw);
        return NULL;
    }
    
    sw->window_size = window_size;
    sw->index = 0;
    sw->count = 0;
    sw->last_sum = 0.0;
    
    return sw;
}

void as5600_sliding_window_destroy(as5600_sliding_window_t *sw) {
    if (sw) {
        if (sw->samples) {
            free(sw->samples);
        }
        free(sw);
    }
}

int as5600_sliding_window_resize(as5600_sliding_window_t *sw, uint16_t new_window_size) {
    if (sw == NULL || new_window_size == 0) {
        return -1;
    }
    
    if (sw->window_size != new_window_size) {
        free(sw->samples);
        sw->samples = calloc(new_window_size, sizeof(float));
        if (sw->samples == NULL) {
            printf("%s: Failed to allocate buffer\n", TAG);
            return -1;
        }
        sw->window_size = new_window_size;
        sw->index = 0;
        sw->count = 0;
        sw->last_sum = 0.0;
    }
    
    return 0;
}

void as5600_sliding_window_clear(as5600_sliding_window_t *sw) {
    if (sw == NULL) {
        return;
    }

    if (sw->samples) {
        memset(sw->samples, 0, sw->window_size * sizeof(float));
    }

    sw->index = 0;
    sw->count = 0;
    sw->last_sum = 0.0;
}

esp_err_t as5600_read_angle_radians(as5600_handle_t handle, float *radians)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(radians != NULL, ESP_ERR_INVALID_ARG, TAG, "Radians pointer is NULL");
    
    uint16_t angle;
    ESP_RETURN_ON_ERROR(as5600_read_angle(handle, &angle), TAG, "Failed to read angle");
    
    *radians = as5600_raw_to_radians(angle);
    return ESP_OK;
}

esp_err_t as5600_read_status(as5600_handle_t handle, uint8_t *status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "Status pointer is NULL");
    
    return as5600_read_reg_8(handle, AS5600_REG_STATUS, status);
}

esp_err_t as5600_read_agc(as5600_handle_t handle, uint8_t *agc)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(agc != NULL, ESP_ERR_INVALID_ARG, TAG, "AGC pointer is NULL");
    
    return as5600_read_reg_8(handle, AS5600_REG_AGC, agc);
}

esp_err_t as5600_read_magnitude(as5600_handle_t handle, uint16_t *magnitude)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(magnitude != NULL, ESP_ERR_INVALID_ARG, TAG, "Magnitude pointer is NULL");
    
    return as5600_read_reg_16(handle, AS5600_REG_MAGNITUDE_H, magnitude);
}

// ====================================================================================
// PUBLIC FUNCTIONS - MAGNET DETECTION AND DIAGNOSTICS
// ====================================================================================

as5600_magnet_status_t as5600_get_magnet_status(as5600_handle_t handle)
{
    if (handle == NULL) {
        return AS5600_MAGNET_NOT_DETECTED;
    }
    
    uint8_t status;
    if (as5600_read_status(handle, &status) != ESP_OK) {
        return AS5600_MAGNET_NOT_DETECTED;
    }
    
    if (status & AS5600_STATUS_MD) {
        return AS5600_MAGNET_TOO_WEAK;
    }
    
    if (status & AS5600_STATUS_ML) {
        return AS5600_MAGNET_TOO_STRONG;
    }
    
    return AS5600_MAGNET_OK;
}

bool as5600_is_magnet_detected(as5600_handle_t handle)
{
    if (handle == NULL) {
        return false;
    }
    
    uint8_t status;
    if (as5600_read_status(handle, &status) != ESP_OK) {
        return false;
    }
    
    return !(status & AS5600_STATUS_MD);
}

uint8_t as5600_get_signal_quality(as5600_handle_t handle)
{
    if (handle == NULL) {
        return 0;
    }
    
    as5600_data_t data;
    if (as5600_read_data(handle, &data) != ESP_OK) {
        return 0;
    }
    
    // Calculate quality based on magnitude and AGC
    uint8_t magnitude_quality = 0;
    uint8_t agc_quality = 0;
    
    // Magnitude quality (0-100%)
    if (data.magnitude >= 100 && data.magnitude <= 2000) {
        magnitude_quality = 100;
    } else if (data.magnitude >= 50 && data.magnitude < 100) {
        magnitude_quality = 50;
    } else if (data.magnitude > 2000 && data.magnitude <= 3000) {
        magnitude_quality = 70;
    } else {
        magnitude_quality = 0;
    }
    
    // AGC quality (0-100%)
    if (data.agc >= 50 && data.agc <= 200) {
        agc_quality = 100;
    } else if (data.agc < 50 || data.agc > 200) {
        agc_quality = 50;
    } else {
        agc_quality = 0;
    }
    
    // Overall quality
    return (magnitude_quality + agc_quality) / 2;
}

esp_err_t as5600_print_diagnostics(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    as5600_data_t data;
    ESP_RETURN_ON_ERROR(as5600_read_data(handle, &data), TAG, "Failed to read data");
    
    ESP_LOGI(TAG, "=== AS5600 Diagnostics ===");
    ESP_LOGI(TAG, "Raw Angle: %d (%.2f°)", data.raw_angle, as5600_raw_to_degrees(data.raw_angle));
    ESP_LOGI(TAG, "Filtered Angle: %d (%.2f°)", data.angle, data.angle_degrees);
    ESP_LOGI(TAG, "Magnitude: %d", data.magnitude);
    ESP_LOGI(TAG, "AGC: %d", data.agc);
    ESP_LOGI(TAG, "Status: 0x%02X", data.status);
    
    // Magnet status
    as5600_magnet_status_t mag_status = as5600_get_magnet_status(handle);
    switch (mag_status) {
        case AS5600_MAGNET_OK:
            ESP_LOGI(TAG, "Magnet Status: OK");
            break;
        case AS5600_MAGNET_TOO_WEAK:
            ESP_LOGW(TAG, "Magnet Status: Too Weak");
            break;
        case AS5600_MAGNET_TOO_STRONG:
            ESP_LOGW(TAG, "Magnet Status: Too Strong");
            break;
        case AS5600_MAGNET_NOT_DETECTED:
            ESP_LOGE(TAG, "Magnet Status: Not Detected");
            break;
    }
    
    // Signal quality
    uint8_t quality = as5600_get_signal_quality(handle);
    ESP_LOGI(TAG, "Signal Quality: %d%%", quality);
    
    // Recommendations
    if (data.magnitude < 50) {
        ESP_LOGW(TAG, "Recommendation: Move magnet closer to sensor");
    } else if (data.magnitude > 2000) {
        ESP_LOGW(TAG, "Recommendation: Move magnet away from sensor");
    }
    
    if (data.agc > 200) {
        ESP_LOGW(TAG, "Recommendation: Increase magnetic field strength");
    } else if (data.agc < 50) {
        ESP_LOGW(TAG, "Recommendation: Decrease magnetic field strength");
    }
    
    ESP_LOGI(TAG, "========================");
    
    return ESP_OK;
}

// ====================================================================================
// PUBLIC FUNCTIONS - CONFIGURATION AND CALIBRATION
// ====================================================================================

esp_err_t as5600_set_zero_position(as5600_handle_t handle, uint16_t position)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(position <= AS5600_MAX_VALUE, ESP_ERR_INVALID_ARG, TAG, "Position out of range");
    
    ESP_LOGI(TAG, "Setting zero position: %d (%.2f°)", position, as5600_raw_to_degrees(position));
    
    return as5600_write_reg_16(handle, AS5600_REG_ZPOS_H, position);
}

esp_err_t as5600_set_max_position(as5600_handle_t handle, uint16_t position)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(position <= AS5600_MAX_VALUE, ESP_ERR_INVALID_ARG, TAG, "Position out of range");
    
    ESP_LOGI(TAG, "Setting max position: %d (%.2f°)", position, as5600_raw_to_degrees(position));
    
    return as5600_write_reg_16(handle, AS5600_REG_MPOS_H, position);
}

esp_err_t as5600_set_max_angle(as5600_handle_t handle, uint16_t max_angle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(max_angle <= AS5600_MAX_VALUE, ESP_ERR_INVALID_ARG, TAG, "Max angle out of range");
    
    ESP_LOGI(TAG, "Setting max angle: %d (%.2f°)", max_angle, as5600_raw_to_degrees(max_angle));
    
    return as5600_write_reg_16(handle, AS5600_REG_MANG_H, max_angle);
}

esp_err_t as5600_set_current_as_zero(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    uint16_t current_angle;
    ESP_RETURN_ON_ERROR(as5600_read_raw_angle(handle, &current_angle), TAG, "Failed to read current angle");
    
    ESP_LOGI(TAG, "Setting current position as zero: %d (%.2f°)", current_angle, as5600_raw_to_degrees(current_angle));
    
    return as5600_set_zero_position(handle, current_angle);
}

esp_err_t as5600_reset_configuration(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    ESP_LOGI(TAG, "Resetting configuration to defaults");
    
    ESP_RETURN_ON_ERROR(as5600_set_zero_position(handle, 0), TAG, "Failed to reset zero position");
    ESP_RETURN_ON_ERROR(as5600_set_max_position(handle, 0), TAG, "Failed to reset max position");
    ESP_RETURN_ON_ERROR(as5600_set_max_angle(handle, 0), TAG, "Failed to reset max angle");
    
    ESP_LOGI(TAG, "Configuration reset complete");
    
    return ESP_OK;
}

esp_err_t as5600_read_configuration(as5600_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    uint16_t zpos, mpos, mang, conf;
    
    ESP_LOGI(TAG, "=== AS5600 Configuration ===");
    
    if (as5600_read_reg_16(handle, AS5600_REG_ZPOS_H, &zpos) == ESP_OK) {
        ESP_LOGI(TAG, "Zero Position (ZPOS): %d (%.2f°)", zpos, as5600_raw_to_degrees(zpos));
    } else {
        ESP_LOGE(TAG, "Failed to read ZPOS");
    }
    
    if (as5600_read_reg_16(handle, AS5600_REG_MPOS_H, &mpos) == ESP_OK) {
        ESP_LOGI(TAG, "Max Position (MPOS): %d (%.2f°)", mpos, as5600_raw_to_degrees(mpos));
    } else {
        ESP_LOGE(TAG, "Failed to read MPOS");
    }
    
    if (as5600_read_reg_16(handle, AS5600_REG_MANG_H, &mang) == ESP_OK) {
        ESP_LOGI(TAG, "Max Angle (MANG): %d (%.2f°)", mang, as5600_raw_to_degrees(mang));
    } else {
        ESP_LOGE(TAG, "Failed to read MANG");
    }
    
    if (as5600_read_reg_16(handle, AS5600_REG_CONF_H, &conf) == ESP_OK) {
        ESP_LOGI(TAG, "Configuration: 0x%04X", conf);
    } else {
        ESP_LOGE(TAG, "Failed to read CONF");
    }
    
    ESP_LOGI(TAG, "===========================");
    
    return ESP_OK;
}

// ====================================================================================
// PUBLIC FUNCTIONS - OTP PROGRAMMING
// ====================================================================================

esp_err_t as5600_burn_settings(as5600_handle_t handle, bool burn_angle, bool burn_config)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    
    ESP_LOGW(TAG, "WARNING: OTP programming is IRREVERSIBLE!");
    ESP_LOGW(TAG, "This will permanently store settings in the AS5600");
    
    // Check current OTP status first
    bool angle_burned, config_burned;
    ESP_RETURN_ON_ERROR(as5600_check_otp_status(handle, &angle_burned, &config_burned), TAG, "Failed to check OTP status");
    
    if ((burn_angle && angle_burned) || (burn_config && config_burned)) {
        ESP_LOGE(TAG, "OTP memory already programmed!");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (burn_angle) {
        ESP_LOGW(TAG, "Burning angle settings to OTP...");
        ESP_RETURN_ON_ERROR(as5600_write_reg_8(handle, AS5600_REG_BURN, 0x80), TAG, "Failed to burn angle settings");
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for burn to complete
    }
    
    if (burn_config) {
        ESP_LOGW(TAG, "Burning configuration settings to OTP...");
        ESP_RETURN_ON_ERROR(as5600_write_reg_8(handle, AS5600_REG_BURN, 0x40), TAG, "Failed to burn config settings");
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for burn to complete
    }
    
    ESP_LOGI(TAG, "OTP programming completed");
    
    return ESP_OK;
}

esp_err_t as5600_check_otp_status(as5600_handle_t handle, bool *angle_burned, bool *config_burned)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
    ESP_RETURN_ON_FALSE(angle_burned != NULL, ESP_ERR_INVALID_ARG, TAG, "Angle burned pointer is NULL");
    ESP_RETURN_ON_FALSE(config_burned != NULL, ESP_ERR_INVALID_ARG, TAG, "Config burned pointer is NULL");
    
    // This is a simplified check - real implementation would need to read specific OTP status registers
    // For now, we assume OTP is not burned (this would need to be implemented based on datasheet)
    *angle_burned = false;
    *config_burned = false;
    
    ESP_LOGI(TAG, "OTP Status - Angle: %s, Config: %s", 
             *angle_burned ? "Burned" : "Not burned",
             *config_burned ? "Burned" : "Not burned");
    
    return ESP_OK;
}