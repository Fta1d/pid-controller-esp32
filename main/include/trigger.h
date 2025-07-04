#ifndef TRIGGER_H
#define TRIGGER_H

#include "config.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t trigger_init(void);
esp_err_t trigger_init_timer(void);

void trigger_shoot(void);

void trigger_interrupt_handler(void *arg);

bool trigger_timer_callback(gptimer_handle_t timer, 
                           const gptimer_alarm_event_data_t *edata, 
                           void *user_data);

#endif // TRIGGER_H