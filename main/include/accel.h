#ifndef ACCEL_H
#define ACCEL_H

#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define LSM303DLHC_WHO_AM_I_REG     0x0F
#define LSM303DLHC_MAG_ADDR         0x1E    // 00011110b = 0x1E
#define LSM303DLHC_ACCEL_ADDR       0x19
#define CRA_REG_M                   0x00
#define CRB_REG_M                   0x01
#define MR_REG_M                    0x02
#define OUT_X_H_M_REG               0x03    // X axis low byte
#define OUT_Y_H_M_REG               0x07    // Y axis low byte
#define OUT_Z_H_M_REG               0x05    // Z axis low byte

#define FILTER_WINDOW_SIZE          10

esp_err_t lsm303_init();
esp_err_t lsm303_read_accel(int16_t *x, int16_t *y, int16_t *z);
esp_err_t lsm303_read_accel_filtered(float *ax, float *ay, float *az);

typedef struct {
    float ax_buffer[FILTER_WINDOW_SIZE];
    float ay_buffer[FILTER_WINDOW_SIZE];
    float az_buffer[FILTER_WINDOW_SIZE];
    int buffer_index;
    bool buffer_full;

    float ax_filtered;
    float ay_filtered; 
    float az_filtered;
    bool initialized;
} accel_filter_t;

#endif // ACCEL_H