/**
 * @file as5600.h
 * @brief AS5600 Magnetic Rotary Position Sensor Driver for ESP-IDF
 * 
 * This library provides a complete interface for the AS5600 magnetic encoder
 * including configuration, calibration, and diagnostic functions.
 * 
 * @author ESP-IDF AS5600 Library
 * @version 1.0.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AS5600 I2C address
 */
#define AS5600_I2C_ADDR                 0x36

/**
 * @brief AS5600 register addresses
 */
#define AS5600_REG_ZPOS_H               0x01    ///< Start position MSB
#define AS5600_REG_ZPOS_L               0x02    ///< Start position LSB
#define AS5600_REG_MPOS_H               0x03    ///< Stop position MSB
#define AS5600_REG_MPOS_L               0x04    ///< Stop position LSB
#define AS5600_REG_MANG_H               0x05    ///< Maximum angle MSB
#define AS5600_REG_MANG_L               0x06    ///< Maximum angle LSB
#define AS5600_REG_CONF_H               0x07    ///< Configuration MSB
#define AS5600_REG_CONF_L               0x08    ///< Configuration LSB
#define AS5600_REG_RAW_ANGLE_H          0x0C    ///< Raw angle MSB
#define AS5600_REG_RAW_ANGLE_L          0x0D    ///< Raw angle LSB
#define AS5600_REG_ANGLE_H              0x0E    ///< Angle MSB
#define AS5600_REG_ANGLE_L              0x0F    ///< Angle LSB
#define AS5600_REG_STATUS               0x0B    ///< Status register
#define AS5600_REG_AGC                  0x1A    ///< Automatic Gain Control
#define AS5600_REG_MAGNITUDE_H          0x1B    ///< Magnitude MSB
#define AS5600_REG_MAGNITUDE_L          0x1C    ///< Magnitude LSB
#define AS5600_REG_BURN                 0xFF    ///< Burn command

/**
 * @brief AS5600 status register bits
 */
#define AS5600_STATUS_MH                (1 << 3)    ///< Magnet too high
#define AS5600_STATUS_ML                (1 << 4)    ///< Magnet too low/strong
#define AS5600_STATUS_MD                (1 << 5)    ///< Magnet detected

/**
 * @brief AS5600 configuration values
 */
#define AS5600_MAX_VALUE                4095        ///< Maximum 12-bit value
#define AS5600_DEGREES_PER_COUNT        (360.0f / 4096.0f)
#define AS5600_RADIANS_PER_COUNT        (2.0f * 3.14159265359f / 4096.0f)

/**
 * @brief AS5600 handle type
 */
typedef struct as5600_dev_t* as5600_handle_t;

/**
 * @brief AS5600 power mode
 */
typedef enum {
    AS5600_POWER_MODE_NOM = 0x00,      ///< Normal mode
    AS5600_POWER_MODE_LPM1 = 0x01,     ///< Low power mode 1
    AS5600_POWER_MODE_LPM2 = 0x02,     ///< Low power mode 2
    AS5600_POWER_MODE_LPM3 = 0x03,     ///< Low power mode 3
} as5600_power_mode_t;

/**
 * @brief AS5600 hysteresis settings
 */
typedef enum {
    AS5600_HYST_OFF = 0x00,            ///< Hysteresis off
    AS5600_HYST_1_LSB = 0x01,          ///< 1 LSB hysteresis
    AS5600_HYST_2_LSB = 0x02,          ///< 2 LSB hysteresis
    AS5600_HYST_3_LSB = 0x03,          ///< 3 LSB hysteresis
} as5600_hysteresis_t;

/**
 * @brief AS5600 output stage
 */
typedef enum {
    AS5600_OUTPUT_ANALOG = 0x00,       ///< Analog output (full range)
    AS5600_OUTPUT_ANALOG_REDUCED = 0x01, ///< Analog output (reduced range)
    AS5600_OUTPUT_PWM = 0x02,          ///< PWM output
} as5600_output_stage_t;

/**
 * @brief AS5600 PWM frequency
 */
typedef enum {
    AS5600_PWM_FREQ_115HZ = 0x00,      ///< 115 Hz
    AS5600_PWM_FREQ_230HZ = 0x01,      ///< 230 Hz
    AS5600_PWM_FREQ_460HZ = 0x02,      ///< 460 Hz
    AS5600_PWM_FREQ_920HZ = 0x03,      ///< 920 Hz
} as5600_pwm_freq_t;

/**
 * @brief AS5600 slow filter
 */
typedef enum {
    AS5600_SLOW_FILTER_16X = 0x00,     ///< 16x (143Hz)
    AS5600_SLOW_FILTER_8X = 0x01,      ///< 8x (286Hz)
    AS5600_SLOW_FILTER_4X = 0x02,      ///< 4x (572Hz)
    AS5600_SLOW_FILTER_2X = 0x03,      ///< 2x (1144Hz)
} as5600_slow_filter_t;

/**
 * @brief AS5600 fast filter threshold
 */
typedef enum {
    AS5600_FAST_FILTER_NONE = 0x00,    ///< No fast filter
    AS5600_FAST_FILTER_6LSB = 0x01,    ///< 6 LSB
    AS5600_FAST_FILTER_7LSB = 0x02,    ///< 7 LSB
    AS5600_FAST_FILTER_9LSB = 0x03,    ///< 9 LSB
    AS5600_FAST_FILTER_18LSB = 0x04,   ///< 18 LSB
    AS5600_FAST_FILTER_21LSB = 0x05,   ///< 21 LSB
    AS5600_FAST_FILTER_24LSB = 0x06,   ///< 24 LSB
    AS5600_FAST_FILTER_10LSB = 0x07,   ///< 10 LSB
} as5600_fast_filter_t;

/**
 * @brief AS5600 configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;              ///< I2C port number
    gpio_num_t sda_pin;               ///< SDA pin
    gpio_num_t scl_pin;               ///< SCL pin
    uint32_t clk_speed;               ///< I2C clock speed (Hz)
    bool enable_pullups;              ///< Enable internal pullups
    
    // AS5600 configuration
    as5600_power_mode_t power_mode;   ///< Power mode
    as5600_hysteresis_t hysteresis;   ///< Hysteresis setting
    as5600_output_stage_t output_stage; ///< Output stage
    as5600_pwm_freq_t pwm_freq;       ///< PWM frequency
    as5600_slow_filter_t slow_filter; ///< Slow filter
    as5600_fast_filter_t fast_filter; ///< Fast filter threshold
    bool watchdog;                    ///< Enable watchdog
} as5600_config_t;

/**
 * @brief AS5600 data structure
 */
typedef struct {
    uint16_t raw_angle;               ///< Raw angle (0-4095)
    uint16_t angle;                   ///< Filtered angle (0-4095)
    float angle_degrees;              ///< Angle in degrees (0-360)
    float angle_radians;              ///< Angle in radians (0-2π)
    uint8_t status;                   ///< Status register
    uint8_t agc;                      ///< AGC value
    uint16_t magnitude;               ///< Magnitude value
    bool magnet_detected;             ///< Magnet detection status
    bool magnet_too_strong;           ///< Magnet too strong flag
    bool magnet_too_weak;             ///< Magnet too weak flag
} as5600_data_t;

typedef struct {
    float *samples;
    uint16_t window_size;
    uint16_t index;
    uint16_t count;
    float last_sum;
} as5600_sliding_window_t;

/**
 * @brief AS5600 magnet status
 */
typedef enum {
    AS5600_MAGNET_OK = 0,             ///< Magnet is OK
    AS5600_MAGNET_TOO_WEAK,           ///< Magnet too weak
    AS5600_MAGNET_TOO_STRONG,         ///< Magnet too strong
    AS5600_MAGNET_NOT_DETECTED,       ///< Magnet not detected
} as5600_magnet_status_t;

/**
 * @brief Default configuration macro
 */
#define AS5600_DEFAULT_CONFIG() {                    \
    .i2c_port = I2C_NUM_0,                          \
    .sda_pin = GPIO_NUM_21,                         \
    .scl_pin = GPIO_NUM_22,                         \
    .clk_speed = 400000,                            \
    .enable_pullups = true,                         \
    .power_mode = AS5600_POWER_MODE_NOM,            \
    .hysteresis = AS5600_HYST_OFF,                  \
    .output_stage = AS5600_OUTPUT_ANALOG,           \
    .pwm_freq = AS5600_PWM_FREQ_115HZ,              \
    .slow_filter = AS5600_SLOW_FILTER_16X,          \
    .fast_filter = AS5600_FAST_FILTER_NONE,         \
    .watchdog = false,                              \
}

// ====================================================================================
// INITIALIZATION AND DEINITIALIZATION
// ====================================================================================

/**
 * @brief Initialize AS5600 device
 * 
 * @param config Pointer to configuration structure
 * @return AS5600 handle on success, NULL on failure
 */
as5600_handle_t as5600_init(const as5600_config_t *config);

/**
 * @brief Deinitialize AS5600 device
 * 
 * @param handle AS5600 handle
 * @return ESP_OK on success
 */
esp_err_t as5600_deinit(as5600_handle_t handle);

/**
 * @brief Check if AS5600 is connected and responding
 * 
 * @param handle AS5600 handle
 * @return ESP_OK if device is responding
 */
esp_err_t as5600_is_connected(as5600_handle_t handle);

// ====================================================================================
// DATA READING
// ====================================================================================

/**
 * @brief Read all sensor data
 * 
 * @param handle AS5600 handle
 * @param data Pointer to data structure
 * @return ESP_OK on success
 */
esp_err_t as5600_read_data(as5600_handle_t handle, as5600_data_t *data);

/**
 * @brief Read raw angle (0-4095)
 * 
 * @param handle AS5600 handle
 * @param raw_angle Pointer to store raw angle
 * @return ESP_OK on success
 */
esp_err_t as5600_read_raw_angle(as5600_handle_t handle, uint16_t *raw_angle);

/**
 * @brief Read filtered angle (0-4095)
 * 
 * @param handle AS5600 handle
 * @param angle Pointer to store angle
 * @return ESP_OK on success
 */
esp_err_t as5600_read_angle(as5600_handle_t handle, uint16_t *angle);

/**
 * @brief Read angle in degrees (0-360)
 * 
 * @param handle AS5600 handle
 * @param degrees Pointer to store angle in degrees
 * @return ESP_OK on success
 */
esp_err_t as5600_read_angle_degrees(as5600_handle_t handle, float *degrees);

/**
 * @brief Read angle in degrees on average (sample rate)
 * 
 * @param handle AS5600 handle
 * @param degrees Pointer to store angle in degrees
 * @param sample_rate How often do you want to read data
 * @return ESP_OK on success
 */
int as5600_read_angle_degrees_sliding(as5600_handle_t handle, as5600_sliding_window_t *sw, float *degrees);
as5600_sliding_window_t* as5600_sliding_window_create(uint16_t window_size);
void as5600_sliding_window_destroy(as5600_sliding_window_t *sw);
int as5600_sliding_window_resize(as5600_sliding_window_t *sw, uint16_t new_window_size);
void as5600_sliding_window_clear(as5600_sliding_window_t *sw);

/**
 * @brief Read angle in radians (0-2π)
 * 
 * @param handle AS5600 handle
 * @param radians Pointer to store angle in radians
 * @return ESP_OK on success
 */
esp_err_t as5600_read_angle_radians(as5600_handle_t handle, float *radians);

/**
 * @brief Read status register
 * 
 * @param handle AS5600 handle
 * @param status Pointer to store status
 * @return ESP_OK on success
 */
esp_err_t as5600_read_status(as5600_handle_t handle, uint8_t *status);

/**
 * @brief Read AGC value
 * 
 * @param handle AS5600 handle
 * @param agc Pointer to store AGC value
 * @return ESP_OK on success
 */
esp_err_t as5600_read_agc(as5600_handle_t handle, uint8_t *agc);

/**
 * @brief Read magnitude value
 * 
 * @param handle AS5600 handle
 * @param magnitude Pointer to store magnitude
 * @return ESP_OK on success
 */
esp_err_t as5600_read_magnitude(as5600_handle_t handle, uint16_t *magnitude);

// ====================================================================================
// MAGNET DETECTION AND DIAGNOSTICS
// ====================================================================================

/**
 * @brief Get magnet status
 * 
 * @param handle AS5600 handle
 * @return Magnet status
 */
as5600_magnet_status_t as5600_get_magnet_status(as5600_handle_t handle);

/**
 * @brief Check if magnet is detected
 * 
 * @param handle AS5600 handle
 * @return true if magnet is detected
 */
bool as5600_is_magnet_detected(as5600_handle_t handle);

/**
 * @brief Get signal quality (0-100%)
 * 
 * @param handle AS5600 handle
 * @return Signal quality percentage
 */
uint8_t as5600_get_signal_quality(as5600_handle_t handle);

/**
 * @brief Print diagnostic information
 * 
 * @param handle AS5600 handle
 * @return ESP_OK on success
 */
esp_err_t as5600_print_diagnostics(as5600_handle_t handle);

// ====================================================================================
// CONFIGURATION AND CALIBRATION
// ====================================================================================

/**
 * @brief Set zero position (start position)
 * 
 * @param handle AS5600 handle
 * @param position Zero position (0-4095)
 * @return ESP_OK on success
 */
esp_err_t as5600_set_zero_position(as5600_handle_t handle, uint16_t position);

/**
 * @brief Set maximum position (stop position)
 * 
 * @param handle AS5600 handle
 * @param position Maximum position (0-4095)
 * @return ESP_OK on success
 */
esp_err_t as5600_set_max_position(as5600_handle_t handle, uint16_t position);

/**
 * @brief Set maximum angle
 * 
 * @param handle AS5600 handle
 * @param max_angle Maximum angle (0-4095, 0 = full rotation)
 * @return ESP_OK on success
 */
esp_err_t as5600_set_max_angle(as5600_handle_t handle, uint16_t max_angle);

/**
 * @brief Set current position as zero
 * 
 * @param handle AS5600 handle
 * @return ESP_OK on success
 */
esp_err_t as5600_set_current_as_zero(as5600_handle_t handle);

/**
 * @brief Reset configuration to defaults
 * 
 * @param handle AS5600 handle
 * @return ESP_OK on success
 */
esp_err_t as5600_reset_configuration(as5600_handle_t handle);

/**
 * @brief Read current configuration
 * 
 * @param handle AS5600 handle
 * @return ESP_OK on success
 */
esp_err_t as5600_read_configuration(as5600_handle_t handle);

// ====================================================================================
// OTP PROGRAMMING (ONE TIME PROGRAMMABLE)
// ====================================================================================

/**
 * @brief Burn settings to OTP memory (IRREVERSIBLE!)
 * 
 * @param handle AS5600 handle
 * @param burn_angle Burn angle settings
 * @param burn_config Burn configuration settings
 * @return ESP_OK on success
 * 
 * @warning This operation is irreversible! Settings will be permanently stored.
 */
esp_err_t as5600_burn_settings(as5600_handle_t handle, bool burn_angle, bool burn_config);

/**
 * @brief Check if OTP memory has been used
 * 
 * @param handle AS5600 handle
 * @param angle_burned Pointer to store angle burn status
 * @param config_burned Pointer to store config burn status
 * @return ESP_OK on success
 */
esp_err_t as5600_check_otp_status(as5600_handle_t handle, bool *angle_burned, bool *config_burned);

// ====================================================================================
// UTILITY FUNCTIONS
// ====================================================================================

/**
 * @brief Convert raw value to degrees
 * 
 * @param raw_value Raw value (0-4095)
 * @return Angle in degrees (0-360)
 */
static inline float as5600_raw_to_degrees(uint16_t raw_value) {
    return (float)raw_value * AS5600_DEGREES_PER_COUNT;
}

/**
 * @brief Convert raw value to radians
 * 
 * @param raw_value Raw value (0-4095)
 * @return Angle in radians (0-2π)
 */
static inline float as5600_raw_to_radians(uint16_t raw_value) {
    return (float)raw_value * AS5600_RADIANS_PER_COUNT;
}

/**
 * @brief Convert degrees to raw value
 * 
 * @param degrees Angle in degrees (0-360)
 * @return Raw value (0-4095)
 */
static inline uint16_t as5600_degrees_to_raw(float degrees) {
    return (uint16_t)(degrees / AS5600_DEGREES_PER_COUNT);
}

/**
 * @brief Convert radians to raw value
 * 
 * @param radians Angle in radians (0-2π)
 * @return Raw value (0-4095)
 */
static inline uint16_t as5600_radians_to_raw(float radians) {
    return (uint16_t)(radians / AS5600_RADIANS_PER_COUNT);
}

#ifdef __cplusplus
}
#endif