#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>

#define UART_NODE DT_NODELABEL(uart0)
#define RX_BUF_SIZE 64
#define RX_TIMEOUT_US 100000  // 100 ms

const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t input_buf[128];
volatile size_t input_pos = 0;
volatile bool line_ready = false;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
        case UART_RX_RDY:
            for (size_t i = 0; i < evt->data.rx.len; i++) {
                char c = evt->data.rx.buf[evt->data.rx.offset + i];

                // Ecoa o caractere digitado
                uart_tx(uart_dev, &c, 1, SYS_FOREVER_MS);

                // Se for Enter, termina a linha
                if (c == '\r' || c == '\n') {
                    // Pula linha no terminal corretamente
                    char newline[] = "\r\n";
                    uart_tx(uart_dev, newline, strlen(newline), SYS_FOREVER_MS);

                    line_ready = true;
                    input_buf[input_pos] = '\0';
                    input_pos = 0;
                    break;
                } else if (input_pos < sizeof(input_buf) - 1) {
                    input_buf[input_pos++] = c;
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

void main(void)
{
    if (!device_is_ready(uart_dev)) {
        return;
    }

    uart_callback_set(uart_dev, uart_cb, NULL);
    uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), RX_TIMEOUT_US);

    const char *welcome = "Digite uma linha e pressione Enter:\r\n";
    uart_tx(uart_dev, welcome, strlen(welcome), SYS_FOREVER_MS);

    while (1) {
        if (line_ready) {
            char response[160];
            snprintf(response, sizeof(response), "Você escreveu: %s\r\n", input_buf);
            uart_tx(uart_dev, response, strlen(response), SYS_FOREVER_MS);
            line_ready = false;
        }
        k_msleep(100);
    }
}