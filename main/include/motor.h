#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"
#include "esp_err.h"

esp_err_t motor_init_pwm(void);


void motor_stop(motor_t *motor);
void motor_stop_all(void);
void motor_set_speed_analog(motor_t *motor, float speed, bool forward);

uint16_t motor_speed_to_duty(float speed);

void motor_restore_states_gamepad(void);
void motor_restore_states(void);

void motor_set_pass_encoder(bool pass);
void motor_set_analog_mode(bool active);
bool motor_get_pass_encoder(void);

void motor_control_task(void *pvParameters);

#endif // MOTOR_H