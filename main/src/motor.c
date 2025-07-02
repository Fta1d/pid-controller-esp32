#include "motor.h"
#include "encoder.h"
#include "trigger.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>

static const char *TAG = "MOTOR";
static TaskHandle_t motor_control_task_handle = NULL;
static volatile bool pass_encoder = false;

void motor_set_pass_encoder(bool pass) {
    pass_encoder = pass;
    ESP_LOGI(TAG, "Pass encoder mode: %s", pass ? "ENABLED" : "DISABLED");
}

bool motor_get_pass_encoder(void) {
    return pass_encoder;
}

esp_err_t motor_init_pwm(void) {
    ESP_LOGI(TAG, "Initializing PWM for motors...");

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t pwm_channels[] = {
        {
            .channel    = X_IN1_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = X_MOTOR_IN1_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = X_IN2_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = X_MOTOR_IN2_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = Y_IN1_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = Y_MOTOR_IN1_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = Y_IN2_LEDC_CHANNEL,
            .duty       = 0,
            .gpio_num   = Y_MOTOR_IN2_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };

    for (size_t i = 0; i < PWM_CHANNELS_NUM; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&pwm_channels[i]));
    }

    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    
    ESP_LOGI(TAG, "PWM initialized successfully");
    return ESP_OK;
}

void motor_stop(motor_channels_t *motor) {
    ledc_set_duty(LEDC_MODE, motor->in1_channel, 0);
    ledc_set_duty(LEDC_MODE, motor->in2_channel, 0);
    ledc_update_duty(LEDC_MODE, motor->in1_channel);
    ledc_update_duty(LEDC_MODE, motor->in2_channel);

    x_state = MOTOR_STOPPED;
    y_state = MOTOR_STOPPED;
}

void motor_stop_all(void) {
    motor_stop(&motor_x_channels);
    x_motor.duty = 0;

    motor_stop(&motor_y_channels);
    y_motor.duty = 0;
}

uint16_t motor_speed_to_duty(float speed) {
    if (speed == 0) return 0;

    uint16_t duty_range = MAX_PWM_DUTY - MIN_PWM_DUTY;
    uint16_t duty = MIN_PWM_DUTY + (speed * duty_range);
    
    return duty;
}

void motor_set_speed_analog(motor_channels_t *motor, float speed, bool forward) {
    // uint16_t duty = motor_speed_to_duty(speed);
    
    static float last_x_speed = 0.0;
    static float last_y_speed = 0.0;
    
    if (motor == &motor_x_channels) {
        motor_state_t desired_state = (speed == 0) ? MOTOR_STOPPED : 
                                     (forward ? MOTOR_BACKWARD : MOTOR_FORWARD);
        
        if (x_state == desired_state && speed > 0 && last_x_speed == speed) {
            ESP_LOGW(TAG, "X param did not change, skipping!");
            return;
        }
        x_state = desired_state;
        last_x_speed = speed;
        
    } else if (motor == &motor_y_channels) {
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
        ledc_set_duty(LEDC_MODE, motor->in1_channel, 0);
        ledc_set_duty(LEDC_MODE, motor->in2_channel, 0);
        ledc_update_duty(LEDC_MODE, motor->in1_channel);
        ledc_update_duty(LEDC_MODE, motor->in2_channel);
        return; 
    }

    if (!forward) {
        ledc_set_duty(LEDC_MODE, motor->in1_channel, speed);
        ledc_set_duty(LEDC_MODE, motor->in2_channel, 0);
    } else {
        ledc_set_duty(LEDC_MODE, motor->in2_channel, speed);
        ledc_set_duty(LEDC_MODE, motor->in1_channel, 0);
    }
    ledc_update_duty(LEDC_MODE, motor->in1_channel);
    ledc_update_duty(LEDC_MODE, motor->in2_channel);

    // ESP_LOGI(TAG, "DUTY %d | %d", (int)ledc_get_duty(LEDC_MODE, motor->in1_channel), (int)ledc_get_duty(LEDC_MODE, motor->in2_channel));
    // ESP_LOGI("MOTOR", "X state %d | Y state %d", (int)x_state, (int)y_state);
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
    // if (motor == 'X') {
    //     if (dir == 0 && angle >= 0) {
    //         ESP_LOGI(TAG, "X motor at max limit (%.2f), cannot move forward", angle);
    //         return false;
    //     }
    //     if (dir == 1 && angle <= 0) {
    //         ESP_LOGI(TAG, "X motor at min limit (%.2f), cannot move backward", angle);
    //         return false;
    //     }
    // } else 
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
    motor_control_task_handle = xTaskGetCurrentTaskHandle();
    const TickType_t motor_control_freq = pdMS_TO_TICKS(20);
    EventBits_t events;

    while (1) {
        TickType_t last_wake_time = xTaskGetTickCount();
        
        if (encoder_read_y_angle() == ESP_OK) {
            // turret_pos.y_angle = y_raw_angle;
        }

        events = xEventGroupWaitBits(
            motor_control_event_group,
            MOTOR_UPDATE_EVENT_X | MOTOR_UPDATE_EVENT_Y | 
            MOTOR_STOP_EVENT | MOTOR_SHOOT_EVENT | TRIGGER_RESTORE_EVENT,
            pdTRUE,  
            pdFALSE,
            0
        );
        
        if (events & MOTOR_UPDATE_EVENT_X) {
            // if (!motor_is_movement_allowed('X', analog_state.x_direction, 0)) {
            //     printf("X motor movement blocked by limit\n");
            //     motor_x.current_freq = 0; 
            //     continue;
            // }

            ESP_LOGI(TAG, "Updating motor X: freq=%d, dir=%d", 
                   (int)x_motor.duty, (int)x_motor.dir);

            motor_set_speed_analog(&motor_x_channels, x_motor.duty, x_motor.dir);
        }
        
        if (events & MOTOR_UPDATE_EVENT_Y) {
            if (!motor_is_movement_allowed('Y', y_motor.dir, y_motor.angle)) {
                ESP_LOGW(TAG, "Y motor movement blocked by limit");
                y_motor.duty = 0;
            } else  {
                ESP_LOGI(TAG, "Updating motor Y: freq=%d, dir=%d", 
                   (int)y_motor.duty, (int)y_motor.dir);
                motor_set_speed_analog(&motor_y_channels, y_motor.duty, y_motor.dir);
            }
        }
        
        if (events & MOTOR_STOP_EVENT) {
            ESP_LOGI(TAG, "Stopping all motors");
            motor_stop_all();
        }
        
        if (events & MOTOR_SHOOT_EVENT) {
            ESP_LOGI(TAG, "Executing shoot sequence");
            trigger_shoot();
        }

        if (events & TRIGGER_RESTORE_EVENT) {
            ESP_LOGI(TAG, "Trigger completed - restoring motors");
            motor_restore_states();
        }
        
        xTaskDelayUntil(&last_wake_time, motor_control_freq);
    }
}