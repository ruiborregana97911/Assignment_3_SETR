#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_NODE DT_NODELABEL(uart0)
#define RX_BUF_SIZE 64
#define FRAME_BUF_SIZE 128
#define TX_BUF_SIZE 128
#define RX_TIMEOUT_US 100000

const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static uint8_t rx_buf[RX_BUF_SIZE];
static char frame_buf[FRAME_BUF_SIZE];
static volatile int frame_pos = 0;
static volatile bool frame_ready = false;
static bool awaiting_enter = false;

int max_temp = 100;
int current_temp = 75;
char controller_params[64] = "Kp=10,Ki=5,Kd=1";
int Kp = 10, Ki = 5, Kd = 1;

uint8_t calc_checksum(const char *cmd, const char *data) {
    uint8_t sum = cmd[0];
    for (size_t i = 0; i < strlen(data); i++) {
        sum += (uint8_t)data[i];
    }
    return sum % 256;
}

void send_uart_msg(const char *msg) {
    while (*msg) {
        uart_tx(uart_dev, msg, 1, SYS_FOREVER_MS);
        msg++;
        k_msleep(1);
    }
}

void process_frame(const char *frame) {
    size_t len = strlen(frame);
    if (len < 5 || frame[0] != '#' || frame[len - 1] != '!') {
        send_uart_msg("\r\n#Ef000!\r\n");
        return;
    }

    char cmd = frame[1];
    char data[64] = {0};
    char cs_str[4] = {0};

    if (cmd == 'C') {
        strncpy(cs_str, &frame[2], 3);
    } else {
        int full_len = strlen(frame);
        int data_len = full_len - 6; // remove # + CMD + CS(3) + !
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
            int t = atoi(data);
            max_temp = t;
            snprintf(response, sizeof(response), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
            send_uart_msg(response);
            break;
        }
        case 'C': {
            char t_str[8] = {0};
            snprintf(t_str, sizeof(t_str), "%03d", current_temp);
            snprintf(response, sizeof(response), "\r\n#c%s%03d!\r\n", t_str, calc_checksum("c", t_str));
            send_uart_msg(response);
            break;
        }
        case 'S': {
            if (strlen(data) != 9 || data[0] != 'p' || data[3] != 'i' || data[6] != 'd') {
                snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
                send_uart_msg(response);
                break;
            }
            char p_str[3] = {data[1], data[2], '\0'};
            char i_str[3] = {data[4], data[5], '\0'};
            char d_str[3] = {data[7], data[8], '\0'};

            int p = atoi(p_str);
            int i = atoi(i_str);
            int d = atoi(d_str);

            if (p < 0 || i < 0 || d < 0 || p > 99 || i > 99 || d > 99) {
                snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
                send_uart_msg(response);
                break;
            }

            Kp = p;
            Ki = i;
            Kd = d;

            snprintf(controller_params, sizeof(controller_params), "Kp=%d,Ki=%d,Kd=%d", Kp, Ki, Kd);
            snprintf(response, sizeof(response), "\r\n#Eo%03d!\r\n", calc_checksum("E", "o"));
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

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    switch (evt->type) {
        case UART_RX_RDY:
            for (size_t i = 0; i < evt->data.rx.len; i++) {
                char c = evt->data.rx.buf[evt->data.rx.offset + i];

                if (c == '\r' || c == '\n') {
                    const char *nl = "\r\n";
                    uart_tx(uart_dev, nl, strlen(nl), SYS_FOREVER_MS);
                } else {
                    uart_tx(uart_dev, &c, 1, SYS_FOREVER_MS);
                }

                if (frame_pos == 0 && c != '#') {
                    frame_buf[frame_pos++] = c;
                    awaiting_enter = true;
                    continue;
                }

                if (awaiting_enter) {
                    if (c == '\r' || c == '\n') {
                        frame_buf[frame_pos] = '\0';
                        frame_ready = true;
                        frame_pos = 0;
                        awaiting_enter = false;
                    }
                    continue;
                }

                if (c == '!') {
                    if (frame_pos < FRAME_BUF_SIZE - 1) {
                        frame_buf[frame_pos++] = c;
                        awaiting_enter = true;
                    } else {
                        send_uart_msg("\r\n#Ef000!\r\n");
                        frame_pos = 0;
                        awaiting_enter = false;
                    }
                } else if (frame_pos < FRAME_BUF_SIZE - 1) {
                    frame_buf[frame_pos++] = c;
                } else {
                    send_uart_msg("\r\n#Ef000!\r\n");
                    frame_pos = 0;
                    awaiting_enter = false;
                }
            }
            break;

        case UART_RX_DISABLED:
            uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT_US);
            break;

        default:
            break;
    }
}

void main(void) {
    if (!device_is_ready(uart_dev)) return;

    uart_callback_set(uart_dev, uart_cb, NULL);
    uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT_US);

    const char *welcome = "Pronto para comandos (#CMD...CS!)\r\n";
    send_uart_msg(welcome);

    while (1) {
        if (frame_ready) {
            process_frame(frame_buf);
            frame_ready = false;
        }
        k_msleep(50);
    }
}
