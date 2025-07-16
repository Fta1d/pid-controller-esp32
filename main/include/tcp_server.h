#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "config.h"

void tcp_server_task(void *pvParameters);

void process_input(char *input);

void tcp_server_task_create(void);

typedef enum {
    TCP_SETX = 0,
    TCP_SETY,
    TCP_STOP,
    TCP_SHOOT,
    TCP_PASS_ENCODER,
    TCP_TARGET,
    TCP_CROSS,
    TCP_AA_SYS,
    TCP_SET,
    TCP_CALIBRATE_BACKLASH,
    TCP_ERR
} cmd_id_tcp_t;

#endif // TCP_SERVER_H