#include "pid.h"
#include "esp_log.h"

static const char *TAG = "PID";

pid_controller_t pid_x = {
    .kp = 20.0,
    .ki = 0.0,
    .kd = 0.0,
    .integral = 0.0,
    .prev_err = 0.0,
    .derivative = 0.0,
    .max_integral = 1000.0,
    .max_output = MAX_PWM_DUTY,
    .min_output = MIN_PWM_DUTY
};

pid_controller_t pid_y = {
    .kp = 20.0,
    .ki = 0.0,
    .kd = 0.0,
    .integral = 0.0,
    .prev_err = 0.0,
    .derivative = 0.0,
    .max_integral = 1000.0,
    .max_output = MAX_PWM_DUTY,
    .min_output = MIN_PWM_DUTY
};

static auto_aim_system_t aa_system = {
    .auto_aim_active = false,
    .target_reached = false,
    .crosshair_x = 0,
    .crosshair_y = 0,
    .target_x = 0,
    .target_y = 0,
    .precision_threshold = 0.8,
    .work_freq = pdMS_TO_TICKS(20)
};

void pid_set_kp(float kp) {
    pid_x.kp = kp;
    pid_y.kp = kp;
}

void pid_set_ki(float ki) {
    pid_x.ki = ki;
    pid_y.ki = ki;
}

void pid_set_kd(float kd) {
    pid_x.kd = kd;
    pid_y.kd = kd;
}

void pid_set_aa_sys_freq_ms(uint16_t ms) {
    aa_system.work_freq = pdMS_TO_TICKS(ms);
}

void pid_set_aa_crosshair_pos(uint16_t x, uint16_t y) {
    aa_system.crosshair_x = x;
    aa_system.crosshair_y = y;
}

void pid_set_aa_target_pos(uint16_t x, uint16_t y) {
    aa_system.target_x = x;
    aa_system.target_y = y;
}

void pid_set_aa_sys_state(bool state) {
    aa_system.auto_aim_active = state;
}

float pid_calculate(pid_controller_t *pid, float error, float dt) {
    float output = 0;

    output += pid->kp * error;

    if (pid->ki > 0) {
        pid->integral += error * dt;
        if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
        if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
        output += pid->ki * pid->integral;
    }

    if (pid->kd > 0) {
        pid->derivative = (error - pid->prev_err) / dt;
        output += pid->kd * pid->derivative;
    }
    
    pid->prev_err = error;

    if (fabs(output) > 0 && fabs(output) < pid->min_output) {
        output = (output > 0) ? pid->min_output : -pid->min_output;
    }

    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;
    
    return output;
}

void auto_aim_target(void) {
    static uint32_t last_time = 0;
    static uint16_t last_x_duty = 0;
    static uint16_t last_y_duty = 0;
    static bool last_x_dir = 0;
    static bool last_y_dir = 0;

    uint32_t current_time = xTaskGetTickCount();
    float dt = (current_time - last_time) * portTICK_PERIOD_MS / 1000.0;

    if (dt < 0.001) dt = 0.001;

    int16_t error_x = aa_system.target_x - aa_system.crosshair_x; 
    int16_t error_y = aa_system.target_y - aa_system.crosshair_y;
    // ESP_LOGI(TAG, "ERROR X: %d | ERROR Y: %d", error_x, error_y);

    if (abs(error_x) < aa_system.precision_threshold && abs(error_y) < aa_system.precision_threshold) {
        if (aa_system.target_reached) return;
        
        xEventGroupSetBits(motor_control_event_group, MOTOR_STOP_EVENT);
        aa_system.target_reached = true;

        last_time = current_time;
        last_x_duty = 0;
        last_y_duty = 0;

        last_x_dir = x_motor.dir;
        last_y_dir = y_motor.dir;
        return;
    }

    aa_system.target_reached = false;

    float output_x = pid_calculate(&pid_x, error_x, dt);
    float output_y = pid_calculate(&pid_y, error_y, dt);

    x_motor.dir = (output_x > 0);
    y_motor.dir = (output_y > 0);

    x_motor.duty = (output_x < 0) ? -output_x : output_x;
    y_motor.duty = (output_y < 0) ? -output_y : output_y;

    if (((x_motor.duty == last_x_duty) && (x_motor.dir == last_x_dir)) 
        && ((y_motor.duty == last_y_duty) && (y_motor.dir == last_y_dir))) {  // Ignoring identical values for less processor time usage
        return;
    } 

    ESP_LOGI(TAG, "DIR X: %d | Y: %d", x_motor.dir, y_motor.dir);
    ESP_LOGI(TAG, "OUTPUT FREQ X: %d | Y: %d", x_motor.duty, y_motor.duty);

    xEventGroupSetBits(motor_control_event_group, MOTOR_UPDATE_EVENT_X | MOTOR_UPDATE_EVENT_Y);

    last_x_duty = x_motor.duty;
    last_y_duty = y_motor.duty;
    last_x_dir  = x_motor.dir;
    last_y_dir  = y_motor.dir;

    last_time = current_time;
}   

static void pid_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        if (aa_system.auto_aim_active) {
            auto_aim_target();
        }

        xTaskDelayUntil(&last_wake_time, aa_system.work_freq);
    }
}

void pid_task_create(void) {
    xTaskCreate(pid_task, "pid", 4096, NULL, 13, NULL);
}