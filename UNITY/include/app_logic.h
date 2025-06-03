#ifndef APP_LOGIC_H
#define APP_LOGIC_H

#include <stdint.h>
#include <stddef.h>

// Definição do tipo RTDB
typedef struct {
    int max_temp;
    int cur_temp;
    float kp, ki, kd;
} rtdb_t;

// Variável global declarada aqui
extern rtdb_t RTDB;

// Funções exportadas
uint8_t calc_checksum(const char *cmd, const char *data);
void print_float_as_string(char *buffer, size_t size, float val);
void process_frame(const char *frame);
void send_uart_msg(const char *msg);

#define PWM_MIN 0.0f
#define PWM_MAX 100.0f
float pid_compute(float kp, float ki, float kd, float *prev, float *intg, float setpoint, float temp, float dt);


#endif
