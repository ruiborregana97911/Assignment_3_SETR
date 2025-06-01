/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/mutex.h>
#include <stdlib.h>
#include <zephyr/drivers/pwm.h>		
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/timing/timing.h>   



#define SLEEP_TIME_MS	1
#define STACK_SIZE 1024
#define PRIORITY_THREAD_TEMP 4
#define PRIORITY_THREAD_LED 5
#define PRIORITY_THREAD_PWM 3
#define PRIORITY_THREAD_UART 5

/* Therad periodicity (in ms)*/
#define THREAD_TEMP_PERIOD 500
#define THREAD_PWM_PERIOD 250
#define THREAD_LED_PERIOD 200

#define PWM_NODE    DT_NODELABEL(pwm_led2)	// heaterpwm is the PWM node for the heater
#define PWM_PERIOD_USEC 1000
#define PWM_MIN 0.0f
#define PWM_MAX 100.0f

#define TC74_CMD_RTR 0x00   /* Read temperature command */
#define TC74_CMD_RWCR 0x01  /* Read/write configuration register */

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)	// sw0 is the button 1
#define SW1_NODE	DT_ALIAS(sw1)	// sw1 is the button 2
#define SW3_NODE	DT_ALIAS(sw3)	// sw3 is the button 4

#define LED0_NODE	DT_ALIAS(led0)	// led0 is the LED 1
#define LED1_NODE	DT_ALIAS(led1)	// led1 is the LED 2
#define LED2_NODE	DT_ALIAS(led2)	// led2 is the LED 3
#define LED3_NODE	DT_ALIAS(led3)	// led3 is the LED 4

/* I2C device vars and defines */
#define I2C0_NID DT_NODELABEL(tc74sensor)

/* UART defs*/
#define UART_NODE DT_NODELABEL(uart0)
#define RX_BUF_SIZE 64
#define FRAME_BUF_SIZE 128
#define TX_BUF_SIZE 128
#define RX_TIMEOUT_US 100000


/* Error codes */
#define ERR_OK  0       // All fine
#define ERR_RDY -1      // Device not ready error
#define ERR_CONF -2     // Configuration error 
#define ERR_ISR -3     // Interrupt error




typedef struct {
    int cur_temp;      // Temperatura lida do sensor
    int setpoint;      // Temperatura desejada
    int max_temp;      // Temperatura máxima de segurança
    bool system_on;    // ON/OFF do sistema
	float kp, ki, kd;	// Parametros PID
	

} rtdb_t;

K_MUTEX_DEFINE(rtdb_mutex);

static rtdb_t RTDB = {
	.cur_temp = 0,
	.setpoint = 30,
	.max_temp = 50,
	.system_on = false, 
	.kp = 5.0f,
	.ki = 0.5f,
	.kd = 0.0f,

	
};

/* UART Vars*/
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static uint8_t rx_buf[RX_BUF_SIZE];
static char frame_buf[FRAME_BUF_SIZE];
static volatile int frame_pos = 0;
static volatile bool frame_ready = false;
static bool awaiting_enter = false;
char controller_params[64] = "Kp=10,Ki=5,Kd=1";



static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button1_cb_data;

static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button2_cb_data;

static const struct gpio_dt_spec button4 = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0});
static struct gpio_callback button4_cb_data;


static struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});
static struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0});
static struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0});
static struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios, {0});

static const struct pwm_dt_spec heater_pwm = PWM_DT_SPEC_GET(PWM_NODE);	

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NID);
							 
/* hardware configurations */
int HWInit(void);

void update_leds(void);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

float pid_compute(float kp,float ki,float kd,float *prev,float *intg, float setpoint, float temp, float dt);

uint8_t calc_checksum(const char *cmd, const char *data);

void send_uart_msg(const char *msg);

void process_frame(const char *frame);

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

void print_float_as_string(char *buffer, size_t size, float val);



/* Thread IDs */
k_tid_t temp_sensor_tid;
k_tid_t led_control_tid;
k_tid_t pwm_tid;

K_THREAD_STACK_DEFINE(temp_sensor_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(led_control_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(pwm_stack, STACK_SIZE);
static struct k_thread temp_sensor_data;
static struct k_thread led_control_data;
static struct k_thread pwm_data;

/* UART thread */
k_tid_t uart_tid;
K_THREAD_STACK_DEFINE(uart_stack, STACK_SIZE);
static struct k_thread uart_data;


void temp_sensor_thread(void *p1, void *p2, void *p3);

void led_control_thread(void *p1, void *p2, void *p3);

void pwm_control_thread(void *p1, void *p2, void *p3);

void uart_task(void *arg1, void *arg2, void *arg3);


int main(void)
{
	int ret = 0;

    if ((ret = HWInit())) {
        printk("HW initialization error!\n");
        return ret;
    }

    printk("RTDB initialized\n");
    printk("System running with threads\n");

    temp_sensor_tid = k_thread_create(&temp_sensor_data, temp_sensor_stack, STACK_SIZE,
                    temp_sensor_thread, NULL, NULL, NULL,
                    PRIORITY_THREAD_TEMP, 0, K_NO_WAIT);

    led_control_tid = k_thread_create(&led_control_data, led_control_stack, STACK_SIZE,
                    led_control_thread, NULL, NULL, NULL,
                    PRIORITY_THREAD_LED, 0, K_NO_WAIT);

	pwm_tid = k_thread_create(&pwm_data, pwm_stack, STACK_SIZE,
		pwm_control_thread, NULL, NULL, NULL,
		PRIORITY_THREAD_PWM, 0, K_NO_WAIT);

	uart_tid = k_thread_create(&uart_data, uart_stack, STACK_SIZE,
					uart_task, NULL, NULL, NULL,
					PRIORITY_THREAD_UART, 0, K_NO_WAIT);


    return 0;
}



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

void print_float_as_string(char *buffer, size_t size, float val) {
    int int_part = (int)val;
    int decimal_part = (int)((val - int_part) * 10);  // 1 casa decimal
    if (decimal_part < 0) decimal_part = -decimal_part;

    snprintf(buffer, size, "%d.%d", int_part, decimal_part);
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

            // Espera-se exatamente 15 chars de dados: pXX.XiXX.XdXX.X
			if (strlen(data) != 12) {
				snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
				send_uart_msg(response);
				break;
			}

			// Validar as posições fixas dos identificadores p, i, d
			if (data[0] != 'p' || data[4] != 'i' || data[8] != 'd') {
				snprintf(response, sizeof(response), "\r\n#Ei%03d!\r\n", calc_checksum("E", "i"));
				send_uart_msg(response);
				break;
			}

			

			// Extrair os valores float (4 chars cada)
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

			// Converte para float dividindo por 10 (último dígito decimal)
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
		case 'G': {  // Get PID parameters
			// Retorna os parâmetros no formato #gpPPP iIII dDDD xxx!
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
                        send_uart_msg("\r\n#Ef171!\r\n");
                        frame_pos = 0;
                        awaiting_enter = false;
                    }
                } else if (frame_pos < FRAME_BUF_SIZE - 1) {
                    frame_buf[frame_pos++] = c;
                } else {
                    send_uart_msg("\r\n#Ef171!\r\n");
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

void uart_task(void *arg1, void *arg2, void *arg3) {
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


void update_leds(void) {
	k_mutex_lock(&rtdb_mutex, K_FOREVER);
    bool on = RTDB.system_on;
    int temp = RTDB.cur_temp;
    int sp = RTDB.setpoint;
    k_mutex_unlock(&rtdb_mutex);

    gpio_pin_set_dt(&led1, on);
    gpio_pin_set_dt(&led2, on && abs(temp - sp) <= 2);
    gpio_pin_set_dt(&led3, on && temp < sp - 2);
    gpio_pin_set_dt(&led4, on && temp > sp + 2);
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_mutex_lock(&rtdb_mutex, K_FOREVER);
    if (pins & BIT(button1.pin)) {
        RTDB.system_on = !RTDB.system_on;
        printk("System is now %s\n", RTDB.system_on ? "ON" : "OFF");

    } else if ((pins & BIT(button2.pin)) && RTDB.system_on) {
		if (RTDB.setpoint < RTDB.max_temp) {
			RTDB.setpoint++;
			printk("Setpoint increased: %d°C\n", RTDB.setpoint);
		} else {
			printk("Setpoint already at maximum: %d°C\n", RTDB.max_temp);
		}

    } else if ((pins & BIT(button4.pin)) && RTDB.system_on) {
        RTDB.setpoint--;
        printk("Setpoint decreased: %d°C\n", RTDB.setpoint);

    }
    k_mutex_unlock(&rtdb_mutex);
    update_leds();
}

void temp_sensor_thread(void *p1, void *p2, void *p3) {

	/* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */  
	
	release_time = k_uptime_get() + THREAD_TEMP_PERIOD;
        

    uint8_t temp = 0;
	int ret=0;
    while (1) {
		k_mutex_lock(&rtdb_mutex, K_FOREVER);
        bool sys_on = RTDB.system_on;
        k_mutex_unlock(&rtdb_mutex);
		if(sys_on){
			/* Read temperature register */       
			ret = i2c_read_dt(&dev_i2c, &temp, sizeof(temp));
			if(ret != 0){
				printk("Failed to read from I2C device at address %x, register  at Reg. %x \n\r", dev_i2c.addr,TC74_CMD_RTR);      
			}
			else{
				k_mutex_lock(&rtdb_mutex, K_FOREVER);
				RTDB.cur_temp = temp;
				k_mutex_unlock(&rtdb_mutex);
				//printk("[SENSOR] Current temperature: %d°C\n", temp);
				//k_sleep(K_MSEC(500));
			}

			
		}
		
		/* Control the task periodicity */
		fin_time = k_uptime_get();
		if (fin_time < release_time) {
			k_msleep(release_time - fin_time);
			release_time += THREAD_TEMP_PERIOD;
		}
    }
	

}

void led_control_thread(void *p1, void *p2, void *p3) {

	/* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */  
	
	release_time = k_uptime_get() + THREAD_LED_PERIOD;

    while (1) {
		k_mutex_lock(&rtdb_mutex, K_FOREVER);
        bool sys_on = RTDB.system_on;
        k_mutex_unlock(&rtdb_mutex);

        if (sys_on) {
            update_leds();
        } else {
            gpio_pin_set_dt(&led1, 0);
            gpio_pin_set_dt(&led2, 0);
            gpio_pin_set_dt(&led3, 0);
            gpio_pin_set_dt(&led4, 0);
        }

        /* Control the task periodicity */
		fin_time = k_uptime_get();
		if (fin_time < release_time) {
			k_msleep(release_time - fin_time);
			release_time += THREAD_LED_PERIOD;
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


void pwm_control_thread(void *p1, void *p2, void *p3) {

	/* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */    
	release_time = k_uptime_get() + THREAD_PWM_PERIOD;

	float prev_error = 0.0f;
	float integral = 0.0f;		

    while (1) {
        k_mutex_lock(&rtdb_mutex, K_FOREVER);
        bool sys_on = RTDB.system_on;
		float sp = (float)RTDB.setpoint;
        float temp = (float)RTDB.cur_temp;
        float kp = RTDB.kp;
        float ki = RTDB.ki;
        float kd = RTDB.kd;
        k_mutex_unlock(&rtdb_mutex);

        if (sys_on) {
            float out = pid_compute(kp, ki, kd, &prev_error, &integral, (float)sp, (float)temp, 0.5f);
            int duty = CLAMP(out, 0, 100);
			//int duty = 50; // For testing, set a fixed duty cycle
            pwm_set_dt(&heater_pwm, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PWM_PERIOD_USEC * duty / 100));
            //printk("[PWM] Duty set to: %d%%\n", duty);
        } else {
            pwm_set_dt(&heater_pwm, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(0));
        }

        /* Control the task periodicity */
		fin_time = k_uptime_get();
		if (fin_time < release_time) {
			k_msleep(release_time - fin_time);
			release_time += THREAD_PWM_PERIOD;
		}

	}


}


int HWInit(void){
	int ret;

	/*button 1 configs*/
	if (!gpio_is_ready_dt(&button1)) {
		printk("Error: button 1 device %s is not ready\n",
		       button1.port->name);
		return ERR_RDY;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure button 1\n",
		       ret);
		return ERR_CONF;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on button 1\n",
			ret);
		return ERR_ISR;
	}

	gpio_init_callback(&button1_cb_data, button_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);
	printk("Set up button 1 at %s pin %d\n", button1.port->name, button1.pin);
	
	/*button 2 configs*/
	if (!gpio_is_ready_dt(&button2)) {
		printk("Error: button 2 device %s is not ready\n",
		       button2.port->name);
		return ERR_RDY;
	}
	ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure button 2\n",
		       ret);
		return ERR_CONF;
	}
	ret = gpio_pin_interrupt_configure_dt(&button2,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on button 2\n",
			ret);
		return ERR_ISR;
	}
	gpio_init_callback(&button2_cb_data, button_pressed, BIT(button2.pin));
	gpio_add_callback(button2.port, &button2_cb_data);
	printk("Set up button 2 at %s pin %d\n", button2.port->name, button2.pin);
	
	/*button 4 configs*/
	if (!gpio_is_ready_dt(&button4)) {
		printk("Error: button 4 device %s is not ready\n",
		       button4.port->name);
		return ERR_RDY;
	}
	ret = gpio_pin_configure_dt(&button4, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure button 4\n",
		       ret);
		return ERR_CONF;
	}
	ret = gpio_pin_interrupt_configure_dt(&button4,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on button 4\n",
			ret);
		return ERR_ISR;
	}
	gpio_init_callback(&button4_cb_data, button_pressed, BIT(button4.pin));
	gpio_add_callback(button4.port, &button4_cb_data);
	printk("Set up button 4 at %s pin %d\n", button4.port->name, button4.pin);


	/*LED 1 configs*/
	if (!gpio_is_ready_dt(&led1)) {
		printk("Error %d: LED 1 device %s is not ready\n",
		       ret, led1.port->name);
		return ERR_RDY;
		
	}
	
	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure LED 1 device %s pin %d\n",
				ret, led1.port->name, led1.pin);
		return ERR_CONF;
	} else {
		printk("Set up LED 1 at %s pin %d\n", led1.port->name, led1.pin);
	}

	/*LED 2 configs*/
	if (!gpio_is_ready_dt(&led2)) {
		printk("Error %d: LED 2 device %s is not ready\n",
		       ret, led2.port->name);
		return ERR_RDY;
		
	}
	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure LED 2 device %s pin %d\n",
				ret, led2.port->name, led2.pin);
		return ERR_CONF;
	} else {
		printk("Set up LED 2 at %s pin %d\n", led2.port->name, led2.pin);
	}

	/*LED 3 configs*/
	if (!gpio_is_ready_dt(&led3)) {
		printk("Error %d: LED 3 device %s is not ready\n",
		       ret, led3.port->name);
		return ERR_RDY;
		
	}
	ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure LED 3 device %s pin %d\n",
				ret, led3.port->name, led3.pin);
		return ERR_CONF;
	} else {
		printk("Set up LED 3 at %s pin %d\n", led3.port->name, led3.pin);
	}

	/*LED 4 configs*/
	if (!gpio_is_ready_dt(&led4)) {
		printk("Error %d: LED 4 device %s is not ready\n",
		       ret, led4.port->name);
		return ERR_RDY;
		
	}
	ret = gpio_pin_configure_dt(&led4, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure LED 4 device %s pin %d\n",
				ret, led4.port->name, led4.pin);
		return ERR_CONF;
	} else {
		printk("Set up LED 4 at %s pin %d\n", led4.port->name, led4.pin);
	}
	
	/* Set the LEDs to off */
	gpio_pin_set_dt(&led1, 0);
	gpio_pin_set_dt(&led2, 0);
	gpio_pin_set_dt(&led3, 0);
	gpio_pin_set_dt(&led4, 0);


	/* PWM configuration */
	if (!device_is_ready(heater_pwm.dev)) {
		printk("Error: PWM device %s is not ready\n", heater_pwm.dev->name);
		return ERR_RDY;
	}
	
	/* I2C configuration*/
	if (!device_is_ready(dev_i2c.bus)) {
	    printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
	    return ERR_RDY;
    } else {
        printk("I2C bus %s, device address %x ready\n\r",dev_i2c.bus->name, dev_i2c.addr);
    }    
    
    /* Write (command RTR) to set the read address to temperature */
    /* Only necessary if a config done before (not the case), but let's stay in the safe side */
    /*ret = i2c_write_dt(&dev_i2c, TC74_CMD_RTR, 1);
    if(ret != 0){
        printk("Failed to write to I2C device at address %x, register %x \n\r", dev_i2c.addr ,TC74_CMD_RTR);
    }*/

	/* UART configuration*/

	if (!device_is_ready(uart_dev)) {
		printk("Error: UART device %s is not ready\n", uart_dev->name);
		return ERR_RDY;
	}

	return ERR_OK;
}










