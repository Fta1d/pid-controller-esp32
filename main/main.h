#ifndef MAIN_H
#define MAIN_H

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "FreeRTOSConfig.h"

#define X_IN1_LEDC_CHANNEL      LEDC_CHANNEL_0
#define X_IN2_LEDC_CHANNEL      LEDC_CHANNEL_1
#define Y_IN1_LEDC_CHANNEL      LEDC_CHANNEL_2
#define Y_IN2_LEDC_CHANNEL      LEDC_CHANNEL_3

#define X_MOTOR_IN1_PIN         GPIO_NUM_21
#define X_MOTOR_IN2_PIN         GPIO_NUM_22
#define Y_MOTOR_IN1_PIN         GPIO_NUM_23
#define Y_MOTOR_IN2_PIN         GPIO_NUM_25

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

// I2C Configuration
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SCL_IO       GPIO_NUM_18
#define I2C_MASTER_SDA_IO       GPIO_NUM_19
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS   1000

// Encoder I2C addresses 
#define X_ENCODER_I2C_ADDR      0x36  // AS5600 default address
#define Y_ENCODER_I2C_ADDR      0x37  // Second encoder 

// AS5600 registers
#define ENCODER_REG_ANGLE_HIGH  0x0E
#define ENCODER_REG_ANGLE_LOW   0x0F
#define ENCODER_REG_RAW_ANGLE_HIGH 0x0C
#define ENCODER_REG_RAW_ANGLE_LOW  0x0D

#define PID_INTERRUPT_X_MOVE    BIT0
#define PID_INTERRUPT_Y_MOVE    BIT1
#define PID_INTERRUPT_X_STOP    BIT2
#define PID_INTERRUPT_Y_STOP    BIT3
#define PID_INTERRUPT_ALL_STOP  BIT4

#define PID_UPDATE_PERIOD       10
#define POSITION_TOLERANCE      0

const float Kp = 1.0;
const float Ki = 0.1;
const float Kd = 0.1;
const float integral_limit = 1000.0; 

volatile uint16_t current_encoder_pos;
EventGroupHandle_t pid_interrupt_group;
SemaphoreHandle_t pid_wake_semaphore;
bool system_brake_engaged = false;

typedef enum {
    MOTOR_STOP,          // Both channels = 0 (free rotaion)
    MOTOR_FORWARD,       // IN1=PWM, IN2=0
    MOTOR_REVERSE,       // IN1=0, IN2=PWM  
    MOTOR_BRAKE          // IN1=MAX, IN2=MAX (magnetic break)
} motor_state_t;

typedef struct {
    float prev_err;
    float integral;
    int16_t target_pos;
    int16_t current_pos;
    uint32_t last_time;
    bool active;
} pid_controller_t;

typedef struct {
    ledc_channel_t in1_channel;
    ledc_channel_t in2_channel;
    pid_controller_t pid;
    volatile int16_t encoder_pos; 
    uint8_t encoder_i2c_addr;
    char name[4];
} motor_t;

motor_t motor_x = {
    .in1_channel = X_IN1_LEDC_CHANNEL,
    .in2_channel = X_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .encoder_i2c_addr = X_ENCODER_I2C_ADDR,
    .name = "X"
};

motor_t motor_y = {
    .in1_channel = Y_IN1_LEDC_CHANNEL,
    .in2_channel = Y_IN2_LEDC_CHANNEL,
    .encoder_pos = 0,
    .encoder_i2c_addr = Y_ENCODER_I2C_ADDR,
    .name = "Y"
};

// Function declarations
esp_err_t i2c_master_init(void);
esp_err_t read_encoder_i2c(uint8_t encoder_addr, int16_t *position);

#endif // MAIN_H