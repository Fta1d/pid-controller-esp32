#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"
#include "esp_err.h"
#include "as5600.h"

esp_err_t encoder_init_dual(void);

esp_err_t encoder_read_y_angle(float *angle);
esp_err_t encoder_read_x_angle(float *angle);

esp_err_t encoder_calibrate_turret(void);

void encoder_deinit(void);

esp_err_t encoder_get_status(void);

extern as5600_handle_t x_encoder;
extern as5600_handle_t y_encoder;

#endif // ENCODER_H