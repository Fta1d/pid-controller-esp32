#include "main.h"

void configure_uart() {
    uart_config_t uart_config = {
        .baud_rate           = 115200,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFF_SIZE * 2, 0, 0, NULL, 0));
}

void configure_pwm() {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_CLK
    };
    ledc_timer_config(&timer_config);

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
        ledc_channel_config(&pwm_channels[i]);
    }

    ledc_fade_func_install(0);
}

void motor_control_task(void *pvParameters) {

}

void process_input(char *input) {
    uint16_t len = strcspn(input, "\r\n");
    char *parsed_input = (char *)malloc(len + 1);
    
    memcpy(parsed_input, input, len);
    parsed_input[len] = '\0';

    char *ptr = parsed_input;
    
    if (*ptr != 0x30) {
        controls.up_pressed = true;
        // ESP_LOGI("pars", "Up pressed");
    } else if (*ptr) {
        // ESP_LOGI("pars", "Up released");
        controls.up_pressed = false;
    }

    if (*(ptr + 1) != 0x30) {
        controls.down_pressed = true;
        // ESP_LOGI("pars", "Down pressed");
    } else if (*(ptr + 1)){
        // ESP_LOGI("pars", "Down released");
        controls.down_pressed = false;
    }

    if (*(ptr + 2) != 0x30) {
        controls.left_pressed = true;
        // ESP_LOGI("pars", "Left pressed"); 
    } else if (*(ptr + 2)){
        // ESP_LOGI("pars", "Left released"); 
        controls.left_pressed = false;
    }

    if (*(ptr + 3) != 0x30) {
        controls.right_pressed = true;
        // ESP_LOGI("pars", "Right pressed");
    } else if (*(ptr + 3)) {
        // ESP_LOGI("pars", "Right released");
        controls.right_pressed = false;
    }

    ESP_LOGI("debug", "%s %d", parsed_input, strlen(parsed_input));
    // uart_write_bytes(UART_NUM, parsed_input, strlen(parsed_input));

    free(parsed_input);
}

void uart_task(void *pvParameters) {
    const char* welcome_msg = "\r\n=== ESP32 PWM Controller ===\r\n> ";

    uart_write_bytes(UART_NUM, welcome_msg, strlen(welcome_msg));

    uint8_t data[UART_BUFF_SIZE];

    while (1) {
        size_t len = uart_read_bytes(UART_NUM, data, UART_BUFF_SIZE - 1, pdMS_TO_TICKS(10));
        data[len] = '\0';

        if (len) {
            // ESP_LOGI("UART", "%s", (char *)data);
            process_input((char *)data);
        }
    }
}

void app_main(void) {
    configure_pwm();
    configure_uart();

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL);
}