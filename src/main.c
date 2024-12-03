/* * Copyright (c) 2016 Intel Corporation * * SPDX-License-Identifier: Apache-2.0 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led2)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

int main(void){ 
	int ret0, ret1; 
 	if (!gpio_is_ready_dt(&led0)) { 
		return 0; 
	}
	ret0 = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);

	if (ret0 < 0) { return 0; }

	if (!gpio_is_ready_dt(&led1)) { return 0; }
	
	ret1 = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE); if (ret1 < 0) { return 0; }

	while (1) { 
		ret0 = gpio_pin_toggle_dt(&led0); 
		ret1 = gpio_pin_toggle_dt(&led1);

		if (ret0 < 0) { 
			return 0; 
		} 
		if (ret1 < 0) { 
			return 0; 
		}

		k_msleep(SLEEP_TIME_MS); 	
	} 
	return 0;
}