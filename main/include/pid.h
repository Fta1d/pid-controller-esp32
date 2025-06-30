#ifndef PID_H
#define PID_H

#include "config.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
    uint16_t max_output;
    uint16_t min_output;

    float kp;
    float ki;
    float kd;

    float integral;
    float prev_err;
    float derivative;

    float max_integral;
} pid_t;

typedef struct {
    bool auto_aim_active;
    bool target_reached;
    uint16_t crosshair_x, crosshair_y;
    uint16_t target_x, target_y;
    const TickType_t work_freq;
    float precision_threshold;
} auto_aim_system_t;

float pid_calculate(pid_t *pid_controller, float error, float dt);
void pid_task_create(void);

void pid_set_kp(float kp);
void pid_set_ki(float ki);
void pid_set_kd(float kd);

extern auto_aim_system_t aa_system;

#endif // PID_H