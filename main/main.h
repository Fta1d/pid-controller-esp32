#ifndef MAIN_H
#define MAIN_H

#include <string.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "FreeRTOSConfig.h"

#define X_IN1_LEDC_CHANNEL      LEDC_CHANNEL_0
#define X_IN2_LEDC_CHANNEL      LEDC_CHANNEL_1
#define Y_IN1_LEDC_CHANNEL      LEDC_CHANNEL_2
#define Y_IN2_LEDC_CHANNEL      LEDC_CHANNEL_3

#define X_MOTOR_IN1_PIN         GPIO_NUM_5
#define X_MOTOR_IN2_PIN         GPIO_NUM_21
#define Y_MOTOR_IN1_PIN         GPIO_NUM_22
#define Y_MOTOR_IN2_PIN         GPIO_NUM_23

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_CLK                LEDC_APB_CLK
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_RES          LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY          4000 // Freq 4 KHz

#define PWM_CHANNELS_NUM        4
#define MAX_PWM_DUTY            4096
#define MIN_PWM_DUTY            50

#define UART_NUM                UART_NUM_2
#define UART_BUFF_SIZE          1024

const int ACCEL_TIME_MS = 100; 

typedef struct {
    ledc_channel_t in1_channel;
    ledc_channel_t in2_channel;
    volatile int16_t encoder_pos; 
    char name[2];
} motor_t;

typedef struct {
    _Atomic bool left_pressed;
    _Atomic bool right_pressed;
    _Atomic bool down_pressed;
    _Atomic bool up_pressed;
} controls_t;

controls_t controls = {
    .left_pressed   = false,
    .right_pressed  = false,
    .down_pressed = false,
    .up_pressed     = false
};

motor_t motor_x = {
    .in1_channel = X_IN1_LEDC_CHANNEL,
    .in2_channel = X_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .name = "X"
};

motor_t motor_y = {
    .in1_channel = Y_IN1_LEDC_CHANNEL,
    .in2_channel = Y_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .name = "Y"
};

#endif // MAIN_H