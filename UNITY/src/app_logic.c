
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "app_logic.h"

rtdb_t RTDB;  // Implementação da variável global



uint8_t calc_checksum(const char *cmd, const char *data) {
    uint8_t sum = cmd[0];
    for (size_t i = 0; i < strlen(data); i++) {
        sum += (uint8_t)data[i];
    }
    return sum % 256;
}

void print_float_as_string(char *buffer, size_t size, float val) {
    int int_part = (int)val;
    int decimal_part = (int)roundf((val - int_part) * 10);

    if (decimal_part >= 10) {
        decimal_part = 0;
        int_part += 1;
    }

    if (decimal_part < 0) decimal_part = -decimal_part;

    snprintf(buffer, size, "%d.%d", int_part, decimal_part);
}

#define TX_BUF_SIZE 128
#define K_FOREVER 0

char rtdb_mutex; // simulado (se estiver usando Zephyr real, troque por struct k_mutex rtdb_mutex)

void k_mutex_lock(void *m, int t) { (void)m; (void)t; }
void k_mutex_unlock(void *m) { (void)m; }

void __attribute__((weak)) send_uart_msg(const char *msg) {
    // implementação vazia fraca; testes podem sobrescrever
}

void process_frame(const char *frame) {
    size_t len = strlen(frame);
    if (len < 5 || frame[0] != '#' || frame[len - 1] != '!') {
        send_uart_msg("\r\n#Ef171!\r\n");
        return;
    }

    char cmd = frame[1];
    char data[64] = {0};
    char cs_str[4] = {0};

    if (cmd == 'C') {
        strncpy(cs_str, &frame[2], 3);
    } else {
        int full_len = strlen(frame);
        int data_len = full_len - 6;
        if (data_len > 0 && data_len < (int)sizeof(data)) {
            strncpy(data, &frame[2], data_len);
            data[data_len] = '\0';
        }
        strncpy(cs_str, &frame[full_len - 4], 3);
    }

    int received_cs = atoi(cs_str);
    char cmd_str[2] = {cmd, '\0'};
    uint8_t computed_cs = calc_checksum(cmd_str, data);

    char response[TX_BUF_SIZE];

    if (computed_cs != received_cs) {
        snprintf(response, sizeof(response), "\r\n#Es%03d!\r\n", calc_checksum("E", "s"));
        send_uart_msg(response);
        return;
    }

    switch (cmd) {
        case 'M': {
            int max_tmp = atoi(data);
            k_mutex_lock(&rtdb_mutex, K_FOREVER);
            RTDB.max_temp = max_tmp;
            k_mutex_unlock(&rtdb_mutex);
            snprintf(response, sizeof(response), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
            send_uart_msg(response);
            snprintf(response, sizeof(response), "Max temp = %d\r\n", max_tmp);
            send_uart_msg(response);
            break;
        }
        case 'C': {
            char t_str[8] = {0};
            int current_temp;
            k_mutex_lock(&rtdb_mutex, K_FOREVER);
            current_temp = RTDB.cur_temp;
            k_mutex_unlock(&rtdb_mutex);
            snprintf(t_str, sizeof(t_str), "%03d", current_temp);
            snprintf(response, sizeof(response), "\r\n#c%s%03d!\r\n", t_str, calc_checksum("c", t_str));
            send_uart_msg(response);
            break;
        }
        case 'S': {
            if (strlen(data) != 12 || data[0] != 'p' || data[4] != 'i' || data[8] != 'd') {
                snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
                send_uart_msg(response);
                break;
            }

            char p_str[5] = { data[1], data[2], data[3], '\0' };
            char i_str[5] = { data[5], data[6], data[7], '\0' };
            char d_str[5] = { data[9], data[10], data[11], '\0' };

            int p_int = atoi(p_str);
            int i_int = atoi(i_str);
            int d_int = atoi(d_str);

            if (p_int < 0 || i_int < 0 || d_int < 0 || p_int > 999 || i_int > 999 || d_int > 999) {
                snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
                send_uart_msg(response);
                break;
            }

            float p = p_int / 10.0f;
            float i = i_int / 10.0f;
            float d = d_int / 10.0f;

            k_mutex_lock(&rtdb_mutex, K_FOREVER);
            RTDB.kp = p;
            RTDB.ki = i;
            RTDB.kd = d;
            k_mutex_unlock(&rtdb_mutex);

            snprintf(response, sizeof(response), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
            send_uart_msg(response);
            break;
        }
        case 'G': {
            float p, i, d;
            k_mutex_lock(&rtdb_mutex, K_FOREVER);
            p = RTDB.kp;
            i = RTDB.ki;
            d = RTDB.kd;
            k_mutex_unlock(&rtdb_mutex);

            int p_int = (int)(p * 10);
            int i_int = (int)(i * 10);
            int d_int = (int)(d * 10);

            char pid_str[20];
            snprintf(pid_str, sizeof(pid_str), "p%03di%03dd%03d", p_int, i_int, d_int);

            snprintf(response, sizeof(response), "\r\n#g%s%03d!\r\n", pid_str, calc_checksum("g", pid_str));
            send_uart_msg(response);
            break;
        }
        default: {
            snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
            send_uart_msg(response);
            break;
        }
    }
}


float pid_compute(float kp,float ki,float kd,float *prev,float *intg, float setpoint, float temp, float dt) {
	
    float error = setpoint - temp;
    //*intg += error * dt;
    float derivative = (error - *prev) / dt;
    
	// Cálculo prévio da saída sem atualizar a integral ainda
    float output_pre = kp * error + ki * (*intg) + kd * derivative;

	// Anti-windup
	if ((output_pre < PWM_MAX && output_pre > PWM_MIN) ||
    	(output_pre >= PWM_MAX && error < 0) ||  
    	(output_pre <= PWM_MIN && error > 0)) {
    	*intg += error * dt;
	}


	float output = kp * error + ki * (*intg) + kd * derivative;

	// Clamp output to PWM range
	if (output < PWM_MIN) {
		output = PWM_MIN;
	} else if (output > PWM_MAX) {
		output = PWM_MAX;
	}

	*prev = error;
    return output;
}
