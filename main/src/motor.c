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
static bool analog_mode_active = false;

void motor_set_pass_encoder(bool pass) {
    pass_encoder = pass;
    ESP_LOGI(TAG, "Pass encoder mode: %s", pass ? "ENABLED" : "DISABLED");
}

bool motor_get_pass_encoder(void) {
    return pass_encoder;
}

void motor_set_analog_mode(bool active) {
    analog_mode_active = active;
    ESP_LOGI(TAG, "Analog mode: %s", active ? "ACTIVE" : "INACTIVE");
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
    // x_state = MOTOR_STOPPED;
}

void motor_stop_all(void) {
    motor_stop(&motor_x_channels);
    motor_stop(&motor_y_channels);
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
            ESP_LOGW(TAG, "In set speed analog IF (X)!");
        }
        x_state = desired_state;
        last_x_speed = speed;
        
    } else if (motor == &motor_y_channels) {
        motor_state_t desired_state = (speed == 0) ? MOTOR_STOPPED : 
                                     (forward ? MOTOR_FORWARD : MOTOR_BACKWARD);

        if (y_state == desired_state && speed > 0 && last_y_speed == speed) {
            ESP_LOGW(TAG, "In set speed analog IF (Y)!");
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
        ledc_update_duty(LEDC_MODE, motor->in1_channel);
    } else {
        ledc_set_duty(LEDC_MODE, motor->in2_channel, speed);
        ledc_update_duty(LEDC_MODE, motor->in2_channel);
    }

    ESP_LOGI("MOTOR", "X state %d | Y state %d", (int)x_state, (int)y_state);
}

void motor_restore_states_gamepad(void) {
    if (x_state != MOTOR_STOPPED) {
        // ESP_LOGI(TAG, "RESTORING X dir: %d, speed: %d", (int)analog_state.x_dir_saved, (int)analog_state.x_speed_saved);

        x_motor.duty = x_motor.duty_saved;
        x_motor.dir = x_motor.dir_saved;

        xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X);
        // motor_set_speed_analog(&motor_x, analog_state.x_speed_saved, analog_state.x_dir_saved);
    }

    if (y_state != MOTOR_STOPPED) {
        // ESP_LOGI(TAG, "RESTORING Y dir: %d, speed: %d", (int)analog_state.y_dir_saved, (int)analog_state.y_speed_saved);

        y_motor.duty = y_motor.duty_saved;
        y_motor.dir = y_motor.dir_saved;

        xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_Y);
        // motor_set_speed_analog(&motor_y, analog_state.y_speed_saved, analog_state.y_dir_saved);
    }
}

// void motor_restore_states(void) {
//     if (x_state == MOTOR_FORWARD) {
//         motor_stop(&motor_x);
//         ledc_set_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL, DUTY);
//         ledc_update_duty(LEDC_MODE, X_IN1_LEDC_CHANNEL);
//     } else if (x_state == MOTOR_BACKWARD) {
//         motor_stop(&motor_x);
//         ledc_set_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL, DUTY);
//         ledc_update_duty(LEDC_MODE, X_IN2_LEDC_CHANNEL);
//     }

//     if (y_state == MOTOR_FORWARD) {
//         motor_stop(&motor_y);
//         ledc_set_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL, DUTY);
//         ledc_update_duty(LEDC_MODE, Y_IN1_LEDC_CHANNEL);
//     } else if (y_state == MOTOR_BACKWARD) {
//         motor_stop(&motor_y);
//         ledc_set_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL, DUTY);
//         ledc_update_duty(LEDC_MODE, Y_IN2_LEDC_CHANNEL);
//     }
// }

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
    } else if (motor == 'Y') {
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
    EventBits_t events;

    while (1) {
        if (trigger_is_shoot_requested()) {
            ESP_LOGI(TAG, "Shoot requested from interrupt");
            trigger_shoot();
        }
        
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            ESP_LOGI(TAG, "Restoring motors after shot");
            motor_restore_states_gamepad();
        }

        if (encoder_read_y_angle() == ESP_OK) {
            // turret_pos.y_angle = y_raw_angle;
        }

        events = xEventGroupWaitBits(
            motor_control_event_group,
            MOTOR_UPDATE_EVENT_X | MOTOR_UPDATE_EVENT_Y | 
            MOTOR_STOP_EVENT | MOTOR_SHOOT_EVENT,
            pdTRUE,  
            pdFALSE,
            pdMS_TO_TICKS(10)
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
                continue;
            }

            ESP_LOGI(TAG, "Updating motor Y: freq=%d, dir=%d", 
                   (int)y_motor.duty, (int)y_motor.dir);

            motor_set_speed_analog(&motor_y_channels, y_motor.duty, y_motor.dir);
        }
        
        if (events & MOTOR_STOP_EVENT) {
            ESP_LOGI(TAG, "Stopping all motors");
            motor_stop_all();
        }
        
        if (events & MOTOR_SHOOT_EVENT) {
            ESP_LOGI(TAG, "Executing shoot sequence");
            trigger_shoot();
        }

        // if (turret_pos.y_angle <= MIN_Y_ANGLE && !pass_encoder) {
        //     motor_stop(&motor_y);
        //     y_state = MOTOR_STOPPED;
        //     ESP_LOGW(TAG, "Y UP blocked - limit: %.2f°", turret_pos.y_angle);
        // } else {
        //     if (y_state != MOTOR_FORWARD) {
        //         ESP_LOGI(TAG, "Y UP: %.2f°", turret_pos.y_angle);
        //         motor_stop(&motor_y);
        //         ledc_set_fade_with_time(LEDC_MODE, Y_IN1_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //         ledc_fade_start(LEDC_MODE, Y_IN1_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //         y_state = MOTOR_FORWARD;
        //     }
        // }

        // if (turret_pos.y_angle >= MAX_Y_ANGLE && !pass_encoder) {
        //     if (y_state == MOTOR_BACKWARD) {
        //         motor_stop(&motor_y);
        //         y_state = MOTOR_STOPPED;
        //         ESP_LOGW(TAG, "Y DOWN blocked - limit: %.2f°", turret_pos.y_angle);
        //     }
        // } else if (y_state != MOTOR_BACKWARD) {
        //     ESP_LOGI(TAG, "Y DOWN: %.2f°", turret_pos.y_angle);
        //     motor_stop(&motor_y);
        //     ledc_set_fade_with_time(LEDC_MODE, Y_IN2_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //     ledc_fade_start(LEDC_MODE, Y_IN2_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //     y_state = MOTOR_BACKWARD;
        // }

        // ESP_LOGI(TAG, "Y Position: %f", turret_pos.y_angle);

        // if (!analog_mode_active) {
        //     bool up = atomic_load_explicit(&controls.up_pressed, memory_order_acquire);
        //     bool down = atomic_load_explicit(&controls.down_pressed, memory_order_acquire);
        //     bool left = atomic_load_explicit(&controls.left_pressed, memory_order_acquire);
        //     bool right = atomic_load_explicit(&controls.right_pressed, memory_order_acquire);
            
        //     // === Y AXIS ===
        //     if (up && !down) {
        //         if (turret_pos.y_angle <= MIN_Y_ANGLE && !pass_encoder) {
        //             motor_stop(&motor_y);
        //             y_state = MOTOR_STOPPED;
        //             ESP_LOGW(TAG, "Y UP blocked - limit: %.2f°", turret_pos.y_angle);
        //         } else {
        //             if (y_state != MOTOR_FORWARD) {
        //                 ESP_LOGI(TAG, "Y UP: %.2f°", turret_pos.y_angle);
        //                 motor_stop(&motor_y);
        //                 ledc_set_fade_with_time(LEDC_MODE, Y_IN1_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //                 ledc_fade_start(LEDC_MODE, Y_IN1_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //                 y_state = MOTOR_FORWARD;
        //             }
        //         }
        //     } else if (down && !up) {
        //         if (turret_pos.y_angle >= MAX_Y_ANGLE && !pass_encoder) {
        //             if (y_state == MOTOR_BACKWARD) {
        //                 motor_stop(&motor_y);
        //                 y_state = MOTOR_STOPPED;
        //                 ESP_LOGW(TAG, "Y DOWN blocked - limit: %.2f°", turret_pos.y_angle);
        //             }
        //         } else if (y_state != MOTOR_BACKWARD) {
        //             ESP_LOGI(TAG, "Y DOWN: %.2f°", turret_pos.y_angle);
        //             motor_stop(&motor_y);
        //             ledc_set_fade_with_time(LEDC_MODE, Y_IN2_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //             ledc_fade_start(LEDC_MODE, Y_IN2_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //             y_state = MOTOR_BACKWARD;
        //         }
        //     } else {
        //         if (y_state == MOTOR_FORWARD || y_state == MOTOR_BACKWARD) {
        //             motor_stop(&motor_y);
        //             y_state = MOTOR_BRAKING;
        //         }
        //     }
            
        //     // === X AXIS ===
        //     if (left && !right) {
        //         if (x_state != MOTOR_FORWARD) {
        //             ESP_LOGI(TAG, "X LEFT: %.2f°", turret_pos.x_angle);
        //             motor_stop(&motor_x);
        //             ledc_set_fade_with_time(LEDC_MODE, X_IN1_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //             ledc_fade_start(LEDC_MODE, X_IN1_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //             x_state = MOTOR_FORWARD;
        //         }
        //     } else if (right && !left) {
        //         if (x_state != MOTOR_BACKWARD) {
        //             ESP_LOGI(TAG, "X RIGHT: %.2f°", turret_pos.x_angle);
        //             motor_stop(&motor_x);
        //             ledc_set_fade_with_time(LEDC_MODE, X_IN2_LEDC_CHANNEL, DUTY, ACCEL_TIME_MS);
        //             ledc_fade_start(LEDC_MODE, X_IN2_LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        //             x_state = MOTOR_BACKWARD;
        //         }
        //     } else {
        //         if (x_state == MOTOR_FORWARD || x_state == MOTOR_BACKWARD) {
        //             motor_stop(&motor_x);
        //             x_state = MOTOR_BRAKING;
        //         }
        //     }
        // }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}