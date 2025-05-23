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

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button1_cb_data;

static bool system_on = false;



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
							 

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button1)) {
		printk("Error: button device %s is not ready\n",
		       button1.port->name);
		return 0;
		}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button1.port->name, button1.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button1.port->name, button1.pin);
		return 0;
	}

	gpio_init_callback(&button1_cb_data, button_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);
	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);

	if (led1.port && !gpio_is_ready_dt(&led1)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led1.port->name);
		led1.port = NULL;
	}
	if (led1.port) {
		ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led1.port->name, led1.pin);
			led1.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led1.port->name, led1.pin);
		}
	}

	printk("Press the button\n");
	if (led1.port) {
		while (1) {
			/* If we have an LED, match its state to the button's. */
			int val = gpio_pin_get_dt(&button1);


			if (val >= 0) {
				gpio_pin_set_dt(&led1, val);
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}
	return 0;
}
