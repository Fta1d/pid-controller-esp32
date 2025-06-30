#ifndef MAIN_H
#define MAIN_H

#include <string.h>
#include <stdatomic.h>
#include <sys/select.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "FreeRTOSConfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "esp_netif.h"

#define X_IN1_LEDC_CHANNEL          LEDC_CHANNEL_0
#define X_IN2_LEDC_CHANNEL          LEDC_CHANNEL_1
#define Y_IN1_LEDC_CHANNEL          LEDC_CHANNEL_2
#define Y_IN2_LEDC_CHANNEL          LEDC_CHANNEL_3

#define X_MOTOR_IN1_PIN             GPIO_NUM_19
#define X_MOTOR_IN2_PIN             GPIO_NUM_18
#define Y_MOTOR_IN1_PIN             GPIO_NUM_23
#define Y_MOTOR_IN2_PIN             GPIO_NUM_5

#define TRIGGER_PIN                 GPIO_NUM_15
#define INPUT_TRIGGER_PIN           GPIO_NUM_34

#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_CLK                    LEDC_APB_CLK
#define LEDC_MODE                   LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_RES              LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY              4000 // Freq 4 KHz

#define PWM_CHANNELS_NUM            4
#define MAX_PWM_DUTY                4095
#define MIN_PWM_DUTY                2500
#define DEFFAULT_DUTY               4095
#define DUTY_STEP                   100

// I2C CONFIGURATION
#define X_ENCODER_I2C_PORT          I2C_NUM_1
#define X_ENCODER_SCL_PIN           GPIO_NUM_4
#define X_ENCODER_SDA_PIN           GPIO_NUM_2

#define Y_ENCODER_I2C_PORT          I2C_NUM_0  
#define Y_ENCODER_SCL_PIN           GPIO_NUM_22
#define Y_ENCODER_SDA_PIN           GPIO_NUM_21

#define I2C_FREQ_HZ                 400000

#define UART_NUM                    UART_NUM_2
#define UART_BUFF_SIZE              1024

#define WIFI_AP_SSID                "esp32"           
#define WIFI_AP_PASS                "turret123"              
#define WIFI_AP_CHANNEL             1                     
#define WIFI_AP_MAX_CONNECTIONS     4  
#define TCP_PORT                    8080

#define MAX_Y_ANGLE                 255.0
#define MIN_Y_ANGLE                 210.0

#define MS_TO_US(ms)                ((ms) * 1000)

#define MIN_SHOOT_TIME              10
#define MAX_SHOOT_TIME              100
#define DEF_SHOOT_TIME              10

const int ACCEL_TIME_MS = 100;

uint16_t DUTY = DEFFAULT_DUTY;
uint16_t shoot_time = DEF_SHOOT_TIME;

gptimer_handle_t shot_timer = NULL;

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

typedef struct {
    float x_speed;      
    float y_speed;
    float x_speed_saved;
    float y_speed_saved;   
    bool x_direction;    
    bool y_direction;
    bool x_dir_saved;
    bool y_dir_saved;      
    bool x_active;         
    bool y_active;         
} analog_motor_state_t;

typedef enum {
    INPUT_MODE_DIGITAL,  
    INPUT_MODE_ANALOG     
} input_mode_t;

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

typedef struct {
    float x_angle;
    float y_angle;
    float target_x;
    float target_y;
} turret_position_t;

extern turret_position_t turret_pos;

#endif // MAIN_H