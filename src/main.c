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


#define SLEEP_TIME_MS	1
#define STACK_SIZE 1024
#define PRIORITY_THREAD_TEMP 5
#define PRIORITY_THREAD_LED 5
#define PRIORITY_THREAD_PWM 5

#define PWM_NODE    DT_ALIAS(pwm_led0) 
#define PWM_PERIOD_USEC 1000

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
	float kp, ki, kd;
	float prev_error;
	float integral;		// PID controller parameters

} rtdb_t;

K_MUTEX_DEFINE(rtdb_mutex);

static rtdb_t RTDB = {
	.cur_temp = 25,
	.setpoint = 30,
	.max_temp = 50,
	.system_on = false, 
	.kp = 5.0f,
	.ki = 0.5f,
	.kd = 0.0f,
	.prev_error = 0.0f,
	.integral = 0.0f
	
};


static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button1_cb_data;

static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button2_cb_data;

static const struct gpio_dt_spec button4 = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0});
static struct gpio_callback button4_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});
static struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0});
static struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0});
static struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios, {0});

static const struct pwm_dt_spec heater_pwm = PWM_DT_SPEC_GET(PWM_NODE);	
							 
/* hardware configurations */
int HWInit(void);

void update_leds(void);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

float pid_compute(float kp,float ki,float kd,float prev,float intg, float setpoint, float temp, float dt);

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

void temp_sensor_thread(void *p1, void *p2, void *p3);

void led_control_thread(void *p1, void *p2, void *p3);

void pwm_control_thread(void *p1, void *p2, void *p3);



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


    return 0;
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
    int fake_temp = 25;
    while (1) {
        fake_temp = (fake_temp >= 40) ? 20 : fake_temp + 1;
        k_mutex_lock(&rtdb_mutex, K_FOREVER);
        RTDB.cur_temp = fake_temp;
        k_mutex_unlock(&rtdb_mutex);
		printk("Current temperature: %d°C\n", fake_temp);
        k_sleep(K_SECONDS(10));
    }
}

void led_control_thread(void *p1, void *p2, void *p3) {
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

        k_sleep(K_MSEC(500));
    }
    
}

/*esta parte nao funciona ver como fazer de forma correta!!!*/

float pid_compute(float setpoint, float temp, float dt) {
	k_mutex_lock(&rtdb_mutex, K_FOREVER);
    float error = RTDB.setpoint - RTDB.cur_temp;
    intg += error * dt;
    float derivative = (error - prev) / dt;
    prev = error;
	k_mutex_unlock(&rtdb_mutex);
    return rtdb->pid.kp * error + rtdb->pid.ki * rtdb->pid.integral + rtdb->pid.kd * derivative;
}


void pwm_control_thread(void *p1, void *p2, void *p3) {
    while (1) {
        k_mutex_lock(&rtdb_mutex, K_FOREVER);
        bool sys_on = RTDB.system_on;
        k_mutex_unlock(&rtdb_mutex);

        if (sys_on) {
            float out = pid_compute((float)sp, (float)temp, 0.5f);
            int duty = CLAMP(out, 0, 100);
            pwm_set_dt(&heater_pwm, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(PWM_PERIOD_USEC * duty / 100));
            printk("[PWM] Duty set to: %d%%\n", duty);
        } else {
            pwm_set_dt(&heater_pwm, PWM_USEC(PWM_PERIOD_USEC), PWM_USEC(0));
        }

        k_sleep(K_MSEC(500));
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



	return ERR_OK;
}










