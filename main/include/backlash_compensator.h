#ifndef BACKLASH_COMPENSATOR_H
#define BACKLASH_COMPENSATOR_H

#include "config.h"
#include "stdlib.h"
#include "math.h"

#include "driver/i2c.h"
#include "esp_log.h"

#define GYRO_THRESHOLD 0.1

typedef struct {
    float backlash_in_angle;
    TickType_t backlash_in_ticks;
    bool ena;
} backlash_compensator_t;

esp_err_t lsm303_mag_init(i2c_port_t port);
void backlash_compensator_task_create();


#endif // BACKLASH_COMPENSATOR_H