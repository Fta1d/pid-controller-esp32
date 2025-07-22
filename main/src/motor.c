#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"

#include "motor.h"
#include "encoder.h"
#include "trigger.h"
#include "backlash_compensator.h"

static const char *TAG = "MOTOR";
static volatile bool pass_encoder = false;

// MCPWM handles
static mcpwm_timer_handle_t x_timer = NULL;
static mcpwm_timer_handle_t y_timer = NULL;
static mcpwm_oper_handle_t x_operator = NULL;
static mcpwm_oper_handle_t y_operator = NULL;
static mcpwm_cmpr_handle_t x_comparator_a = NULL;
static mcpwm_cmpr_handle_t x_comparator_b = NULL;
static mcpwm_cmpr_handle_t y_comparator_a = NULL;
static mcpwm_cmpr_handle_t y_comparator_b = NULL;
static mcpwm_gen_handle_t x_gen_a = NULL;
static mcpwm_gen_handle_t x_gen_b = NULL;
static mcpwm_gen_handle_t y_gen_a = NULL;
static mcpwm_gen_handle_t y_gen_b = NULL;

#define MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define MCPWM_PERIOD_TICKS        120    // 120us = 83kHz frequency

void motor_set_pass_encoder(bool pass) {
    pass_encoder = pass;
    ESP_LOGI(TAG, "Pass encoder mode: %s", pass ? "ENABLED" : "DISABLED");
}

bool motor_get_pass_encoder(void) {
    return pass_encoder;
}

esp_err_t motor_init_pwm(void) {
    ESP_LOGI(TAG, "Initializing MCPWM for motors...");

    mcpwm_timer_config_t x_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MCPWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&x_timer_config, &x_timer));

    mcpwm_timer_config_t y_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MCPWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&y_timer_config, &y_timer));

    mcpwm_operator_config_t x_operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&x_operator_config, &x_operator));

    mcpwm_operator_config_t y_operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&y_operator_config, &y_operator));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(x_operator, x_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(y_operator, y_timer));

    mcpwm_comparator_config_t x_comparator_config = {};
    ESP_ERROR_CHECK(mcpwm_new_comparator(x_operator, &x_comparator_config, &x_comparator_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(x_operator, &x_comparator_config, &x_comparator_b));

    mcpwm_comparator_config_t y_comparator_config = {};
    ESP_ERROR_CHECK(mcpwm_new_comparator(y_operator, &y_comparator_config, &y_comparator_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(y_operator, &y_comparator_config, &y_comparator_b));

    mcpwm_generator_config_t x_gen_a_config = {
        .gen_gpio_num = X_MOTOR_IN1_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(x_operator, &x_gen_a_config, &x_gen_a));

    mcpwm_generator_config_t x_gen_b_config = {
        .gen_gpio_num = X_MOTOR_IN2_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(x_operator, &x_gen_b_config, &x_gen_b));

    mcpwm_generator_config_t y_gen_a_config = {
        .gen_gpio_num = Y_MOTOR_IN1_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(y_operator, &y_gen_a_config, &y_gen_a));

    mcpwm_generator_config_t y_gen_b_config = {
        .gen_gpio_num = Y_MOTOR_IN2_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(y_operator, &y_gen_b_config, &y_gen_b));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(x_gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(x_gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(y_gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(y_gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(x_timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(y_timer));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(x_timer, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(y_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM initialized successfully");
    return ESP_OK;
}

static void motor_set_pwm_duty(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmpr, uint32_t duty) {
    uint32_t duty_ticks = (duty * MCPWM_PERIOD_TICKS) / MAX_PWM_DUTY;
    if (duty_ticks > MCPWM_PERIOD_TICKS) duty_ticks = MCPWM_PERIOD_TICKS;
    
    if (duty == 0) {
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));
    } else {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, duty_ticks));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));
    }
}

void motor_stop(char motor) {
    if (motor == 'X') {
        motor_set_pwm_duty(x_gen_a, x_comparator_a, 0);
        motor_set_pwm_duty(x_gen_b, x_comparator_b, 0);
    } else if (motor == 'Y') {
        motor_set_pwm_duty(y_gen_a, y_comparator_a, 0);
        motor_set_pwm_duty(y_gen_b, y_comparator_b, 0);
    }

    x_state = MOTOR_STOPPED;
    y_state = MOTOR_STOPPED;
}

void motor_stop_all(void) {
    motor_stop('X');
    x_motor.duty = 0;

    motor_stop('Y');
    y_motor.duty = 0;
}

uint16_t motor_speed_to_duty(float speed) {
    if (speed == 0) return 0;

    uint16_t duty_range = MAX_PWM_DUTY - MIN_PWM_DUTY;
    uint16_t duty = MIN_PWM_DUTY + (speed * duty_range);
    
    return duty;
}

void motor_set_speed_analog(char motor, uint32_t speed, bool forward) {
    static uint32_t last_x_speed = 0;
    static uint32_t last_y_speed = 0;
    static bool last_x_direction = true;
    static bool first_x_move = true;
    
    if (motor == 'X') {
        motor_state_t desired_state = (speed == 0) ? MOTOR_STOPPED : 
                                     (forward ? MOTOR_BACKWARD : MOTOR_FORWARD);
        
        if (x_state == desired_state && speed > 0 && last_x_speed == speed) {
            ESP_LOGW(TAG, "X param did not change, skipping!");
            return;
        }

        if (speed > 0) {
            bool direction_changed = !first_x_move && last_x_direction != forward;
            
            if (direction_changed && backlash_compensator_is_enabled()) {
                ESP_LOGI(TAG, "Direction changed: %s -> %s, applying backlash compensation",
                        last_x_direction ? "FORWARD" : "BACKWARD",
                        forward ? "FORWARD" : "BACKWARD");
                
                skip_backlash(forward, speed);
                last_x_speed = 0;
                
                x_state = desired_state;
                last_x_direction = forward;
                first_x_move = false;
                return;  
            }   
            if (!first_x_move || !direction_changed) {
                last_x_direction = forward;
                first_x_move = false;
            }
        }

        x_state = desired_state;
        last_x_speed = speed;
        
    } else if (motor == 'Y') {
        motor_state_t desired_state = (speed == 0) ? MOTOR_STOPPED : 
                                     (forward ? MOTOR_FORWARD : MOTOR_BACKWARD);

        if (y_state == desired_state && speed > 0 && last_y_speed == speed) {
            ESP_LOGW(TAG, "Y param did not change, skipping!");
            return;
        }
        y_state = desired_state;
        last_y_speed = speed;
    }

    if (speed == 0) {
        motor_stop(motor);
        return; 
    }

    if (motor == 'X') {
        if (!forward) {
            // IN1 = PWM, IN2 = 0
            motor_set_pwm_duty(x_gen_a, x_comparator_a, speed);
            motor_set_pwm_duty(x_gen_b, x_comparator_b, 0);
        } else {
            // IN1 = 0, IN2 = PWM
            motor_set_pwm_duty(x_gen_a, x_comparator_a, 0);
            motor_set_pwm_duty(x_gen_b, x_comparator_b, speed);
        }
    } else if (motor == 'Y') {
        if (!forward) {
            // IN1 = PWM, IN2 = 0
            motor_set_pwm_duty(y_gen_a, y_comparator_a, speed);
            motor_set_pwm_duty(y_gen_b, y_comparator_b, 0);
        } else {
            // IN1 = 0, IN2 = PWM
            motor_set_pwm_duty(y_gen_a, y_comparator_a, 0);
            motor_set_pwm_duty(y_gen_b, y_comparator_b, speed);
        }
    }
}

void motor_restore_states(void) {
    if (x_motor.duty_saved != 0) {
        ESP_LOGI(TAG, "RESTORING X dir: %d, speed: %d", (int)x_motor.dir_saved, (int)x_motor.duty_saved);

        x_motor.duty = x_motor.duty_saved;
        x_motor.dir = x_motor.dir_saved;

        xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
    }

    if (y_motor.duty_saved != 0) {
        ESP_LOGI(TAG, "RESTORING Y dir: %d, speed: %d", (int)y_motor.dir_saved, (int)y_motor.duty_saved);

        y_motor.duty = y_motor.duty_saved;
        y_motor.dir = y_motor.dir_saved;

        xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_Y);
    }
}

bool motor_is_movement_allowed(char motor, bool dir, float angle) {
    if (motor == 'X') {
        if (dir == 0 && angle >= 0) {
            ESP_LOGI(TAG, "X motor at max limit (%.2f), cannot move forward", angle);
            return false;
        }
        if (dir == 1 && angle <= 0) {
            ESP_LOGI(TAG, "X motor at min limit (%.2f), cannot move backward", angle);
            return false;
        }
    } else 
    if (motor == 'Y') {
        if (dir == 0 && angle >= MAX_Y_ANGLE) {
            ESP_LOGI(TAG, "Y motor at max limit (%.2f), cannot move forward", angle);
            return false;
        }
        if (dir == 1 && angle <= MIN_Y_ANGLE) {
            ESP_LOGI(TAG, "Y motor at min limit (%.2f), cannot move backward", angle);
            return false;
        }
    }
    return true;
}

void motor_control_task(void *pvParameters) {
    const TickType_t motor_control_freq = pdMS_TO_TICKS(10);
    EventBits_t events;

    while (1) {
        TickType_t last_wake_time = xTaskGetTickCount();

        events = xEventGroupWaitBits(
            motor_control_event_group,
            MOTOR_UPDATE_EVENT_X | MOTOR_UPDATE_EVENT_Y | 
            MOTOR_STOP_EVENT | MOTOR_SHOOT_EVENT | TRIGGER_RESTORE_EVENT,
            pdTRUE,  
            pdFALSE,
            0
        );

        if (backlash_compensation_in_progress()) {
            // Ignoring MOTOR_STOP_EVENT during backlash comp
            events &= ~MOTOR_STOP_EVENT;
        }
        
        if (events & MOTOR_STOP_EVENT) {
            ESP_LOGI(TAG, "Stopping all motors");
            motor_stop_all();
        }

        if (events & MOTOR_SHOOT_EVENT) {
            ESP_LOGI(TAG, "Executing shoot sequence");
            trigger_shoot();
        }

        if (events & MOTOR_UPDATE_EVENT_X) {
            ESP_LOGI(TAG, "Updating motor X: freq=%d, dir=%d", 
                   (int)x_motor.duty, (int)x_motor.dir);

            motor_set_speed_analog('X', x_motor.duty, x_motor.dir);
        }
        
        if (events & MOTOR_UPDATE_EVENT_Y) {
            if (!motor_is_movement_allowed('Y', y_motor.dir, y_motor.angle)) {
                ESP_LOGW(TAG, "Y motor movement blocked by limit");
                y_motor.duty = 0;
            } else  {
                ESP_LOGI(TAG, "Updating motor Y: freq=%d, dir=%d", 
                   (int)y_motor.duty, (int)y_motor.dir);
                motor_set_speed_analog('Y', y_motor.duty, y_motor.dir);
            }
        }

        if (events & TRIGGER_RESTORE_EVENT) {
            ESP_LOGI(TAG, "Trigger completed - restoring motors");
            motor_restore_states();
        }
        
        xTaskDelayUntil(&last_wake_time, motor_control_freq);
    }
}

void motor_task_create(void) {
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 10, NULL);
}