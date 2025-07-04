#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gptimer.h"

// === GPIO PINS ===
#define X_MOTOR_IN1_PIN         GPIO_NUM_19
#define X_MOTOR_IN2_PIN         GPIO_NUM_18
#define Y_MOTOR_IN1_PIN         GPIO_NUM_23
#define Y_MOTOR_IN2_PIN         GPIO_NUM_5
#define TRIGGER_PIN             GPIO_NUM_15
#define INPUT_TRIGGER_PIN       GPIO_NUM_34

// === PWM CONFIGURATION ===
#define X_IN1_LEDC_CHANNEL      LEDC_CHANNEL_0
#define X_IN2_LEDC_CHANNEL      LEDC_CHANNEL_1
#define Y_IN1_LEDC_CHANNEL      LEDC_CHANNEL_2
#define Y_IN2_LEDC_CHANNEL      LEDC_CHANNEL_3
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_CLK                LEDC_APB_CLK
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_RES          LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY          4000
#define PWM_CHANNELS_NUM        4
#define MAX_PWM_DUTY            4095
#define MIN_PWM_DUTY            3000
#define DEFAULT_DUTY            4095
#define DUTY_STEP               100

// === I2C CONFIGURATION ===
#define X_ENCODER_I2C_PORT      I2C_NUM_1
#define X_ENCODER_SCL_PIN       GPIO_NUM_4
#define X_ENCODER_SDA_PIN       GPIO_NUM_2
#define Y_ENCODER_I2C_PORT      I2C_NUM_0
#define Y_ENCODER_SCL_PIN       GPIO_NUM_22
#define Y_ENCODER_SDA_PIN       GPIO_NUM_21
#define I2C_FREQ_HZ             400000

// === UART CONFIGURATION ===
#define UART_NUM                UART_NUM_2
#define UART_BUFF_SIZE          1024

// === WIFI CONFIGURATION ===
#define WIFI_AP_SSID            "esp32"
#define WIFI_AP_PASS            "turret123"
#define WIFI_AP_CHANNEL         1
#define WIFI_AP_MAX_CONNECTIONS 4
#define TCP_PORT                8080

// === MOVEMENT LIMITS ===
#define MAX_Y_ANGLE             138.0f
#define MIN_Y_ANGLE             103.0f
#define MAX_X_ANGLE             0.0f
#define MIN_X_ANGLE             0.0f

// === TIMING ===
#define MS_TO_US(ms)            ((ms) * 1000)
#define MIN_SHOOT_TIME          10
#define MAX_SHOOT_TIME          100
#define DEF_SHOOT_TIME          50

#define MOTOR_UPDATE_EVENT_X    (1 << 0)
#define MOTOR_UPDATE_EVENT_Y    (1 << 1)
#define MOTOR_STOP_EVENT        (1 << 2)
#define MOTOR_SHOOT_EVENT       (1 << 3)
#define TRIGGER_RESTORE_EVENT   (1 << 4)

// === GLOBAL VARIABLES ===
extern uint16_t shoot_time;
extern gptimer_handle_t shot_timer;
extern EventGroupHandle_t motor_control_event_group;

// === ENUMS ===
typedef enum {
    MOTOR_STOPPED,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
} motor_state_t;

typedef enum {
    NOTIFY_TRIGGER_SHOOT = 1,
    NOTIFY_TRIGGER_RESTORE_MOTORS = 2
} trigger_notify_t;

// === STRUCTS ===
typedef struct {
    ledc_channel_t in1_channel;
    ledc_channel_t in2_channel;
    char name[2];
} motor_channels_t;

typedef struct {
    float angle;

    uint16_t duty;
    uint16_t duty_saved;

    bool dir;
    bool dir_saved;
    bool active;
} motor_t;

// === GLOBAL INSTANCES ===
extern motor_channels_t motor_x_channels;
extern motor_channels_t motor_y_channels;
extern motor_t x_motor;
extern motor_t y_motor;
extern motor_state_t y_state;
extern motor_state_t x_state;

#endif // CONFIG_H      