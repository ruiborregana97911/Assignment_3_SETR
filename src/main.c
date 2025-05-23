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

#define SLEEP_TIME_MS	1

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)	// sw0 is the button 1
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

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
} rtdb_t;



static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
	{0});
static struct gpio_callback button1_cb_data;

static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
	{0});
static struct gpio_callback button2_cb_data;

static const struct gpio_dt_spec button4 = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios,
	{0});
static struct gpio_callback button4_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios,
						     {0});
static struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios,
						     {0});
static struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios,
						     {0});
static struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios,
						     {0});
							 
/* hardware configurations */
int HWInit(void);



void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	int ret=0;

    if((ret = HWInit())) {
        printk("HW initialization error!\n");
        return(ret);
    }

	printk("Press the button\n");
	
	while (1) {
		/* If we have an LED, match its state to the button's. */
		int val = gpio_pin_get_dt(&button1);

		if (val >= 0) {
			gpio_pin_set_dt(&led1, val);
		}



		k_msleep(SLEEP_TIME_MS);
	}
	
	return 0;
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










