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
    bool compensation_in_progress; 
} backlash_compensator_t;

void calculate_backlash(/**/);
void backlash_compensator_task_create();
void backlash_comp_enable(bool val);
bool backlash_compensator_is_enabled();
bool backlash_compensation_in_progress(void);
void skip_backlash(bool dir, uint32_t target_speed);


#endif // BACKLASH_COMPENSATOR_H