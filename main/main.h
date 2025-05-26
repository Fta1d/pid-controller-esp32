#ifndef MAIN_H
#define MAIN_H

#include <string.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "FreeRTOSConfig.h"

#define X_IN1_LEDC_CHANNEL          LEDC_CHANNEL_0
#define X_IN2_LEDC_CHANNEL          LEDC_CHANNEL_1
#define Y_IN1_LEDC_CHANNEL          LEDC_CHANNEL_2
#define Y_IN2_LEDC_CHANNEL          LEDC_CHANNEL_3

#define X_MOTOR_IN1_PIN             GPIO_NUM_18
#define X_MOTOR_IN2_PIN             GPIO_NUM_19
#define Y_MOTOR_IN1_PIN             GPIO_NUM_5
#define Y_MOTOR_IN2_PIN             GPIO_NUM_23

#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_CLK                    LEDC_APB_CLK
#define LEDC_MODE                   LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_RES              LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY              4000 // Freq 4 KHz

#define PWM_CHANNELS_NUM            4
#define MAX_PWM_DUTY                4095
#define MIN_PWM_DUTY                50

#define UART_NUM                    UART_NUM_2
#define UART_BUFF_SIZE              1024

// I2C Configuration
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_SCL_IO           GPIO_NUM_18
#define I2C_MASTER_SDA_IO           GPIO_NUM_19
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Encoder I2C addresses 
#define X_ENCODER_I2C_ADDR          0x36  // AS5600 default address
#define Y_ENCODER_I2C_ADDR          0x37  // Second encoder 

// AS5600 registers
#define ENCODER_REG_ANGLE_HIGH      0x0E
#define ENCODER_REG_ANGLE_LOW       0x0F
#define ENCODER_REG_RAW_ANGLE_HIGH  0x0C
#define ENCODER_REG_RAW_ANGLE_LOW   0x0D

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

typedef enum {
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKING
} motor_state_t;

#endif // MAIN_H