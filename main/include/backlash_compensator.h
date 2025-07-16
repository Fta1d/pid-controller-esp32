#ifndef BACKLASH_COMPENSATOR_H
#define BACKLASH_COMPENSATOR_H

#include "config.h"
#include "stdlib.h"
#include "math.h"

#include "esp_log.h"

typedef struct {
    float backlash_in_angle;
    TickType_t backlash_in_ticks;
    bool ena;
} backlash_compensator_t;

void calculate_backlash(/**/);
void backlash_compensator_task_create();


#endif // BACKLASH_COMPENSATOR_H