#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "config.h"

void tcp_server_task(void *pvParameters);

void process_input(char *input);
void parse_analog_command(const char *input);

void reset_current_analog_state(void);

typedef enum {
    SETX = 0,
    SETY,
    STOP,
    SHOOT,
    PASS_ENCODER,
    TARGET,
    CROSS,
    AA_SYS,
    ERR
} cmd_id_t;

#endif // TCP_SERVER_H