/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include "fsl_rdc.h"

#define M7_DOMAIN_ID (1U)
#define A53_DOMAIN_ID (0U)

LOG_MODULE_REGISTER(rgb_blinky, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "red_led". */
#define RED_LED_NODE DT_NODELABEL(red_led)

#if DT_NODE_HAS_STATUS(RED_LED_NODE, okay)
#define RED_LED DT_GPIO_LABEL(RED_LED_NODE, gpios)
#define RED_PIN DT_GPIO_PIN(RED_LED_NODE, gpios)
#define RED_FLAGS DT_GPIO_FLAGS(RED_LED_NODE, gpios)
#else
#error "Unsupported board: red_led devicetree is not defined"
#define RED_LED ""
#define RED_PIN 0
#define RED_FLAGS 0
#endif

/* The devicetree node identifier for the "green_led". */
#define GREEN_LED_NODE DT_NODELABEL(green_led)

#if DT_NODE_HAS_STATUS(GREEN_LED_NODE, okay)
#define GREEN_LED DT_GPIO_LABEL(GREEN_LED_NODE, gpios)
#define GREEN_PIN DT_GPIO_PIN(GREEN_LED_NODE, gpios)
#define GREEN_FLAGS DT_GPIO_FLAGS(GREEN_LED_NODE, gpios)
#else
#error "Unsupported board: green_led devicetree is not defined"
#define GREEN_LED ""
#define GREEN_PIN 0
#define GREEN_FLAGS 0
#endif

/* The devicetree node identifier for the "blue_led". */
#define BLUE_LED_NODE DT_NODELABEL(blue_led)

#if DT_NODE_HAS_STATUS(BLUE_LED_NODE, okay)
#define BLUE_LED DT_GPIO_LABEL(BLUE_LED_NODE, gpios)
#define BLUE_PIN DT_GPIO_PIN(BLUE_LED_NODE, gpios)
#define BLUE_FLAGS DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)
#else
#error "Unsupported board: blue_led is not defined"
#define BLUE_LED ""
#define BLUE_PIN 0
#define BLUE_FLAGS 0
#endif

enum color { red, green, blue, magenta, yellow, cyan, white };

void rdc_configure_peripheral_access(void)
{
	rdc_domain_assignment_t assignment = { 0 };
	rdc_periph_access_config_t periph_config;

	assignment.domainId = M7_DOMAIN_ID;

	/* Only configure the RDC if the RDC peripheral write access is allowed. */
	if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, assignment.domainId)) != 0U) {
		RDC_GetDefaultPeriphAccessConfig(&periph_config);

		/* Give the A53 domain (domain0) access to the I2C3 and SAI3 peripherals. */
		periph_config.policy = RDC_ACCESS_POLICY(A53_DOMAIN_ID, kRDC_ReadWrite);
		periph_config.periph = kRDC_Periph_SAI3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_I2C3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);

		/* Give the M7 domain (domain1) access to the I2C3 and peripheral. */
		periph_config.policy = RDC_ACCESS_POLICY(M7_DOMAIN_ID, kRDC_ReadWrite);
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
	}
}

void main(void)
{
	const struct device *red_dev;
	const struct device *green_dev;
	const struct device *blue_dev;
	enum color current_color = red;
	int ret;

	rdc_configure_peripheral_access();

	LOG_INF("RGB Blinky Demo");

	/* Startup delay */
	k_sleep(K_MSEC(100));

	/* Configure the red LED */
	red_dev = device_get_binding(RED_LED);
	if (red_dev == NULL) {
		LOG_ERR("Unable to get device binding for the red LED!");
		return;
	}

	ret = gpio_pin_configure(red_dev, RED_PIN, GPIO_OUTPUT_ACTIVE | RED_FLAGS);
	if (ret < 0) {
		LOG_ERR("Unable to configure GPIO pin for the red LED!");
		return;
	}

	/* Configure the green LED */
	green_dev = device_get_binding(GREEN_LED);
	if (green_dev == NULL) {
		LOG_ERR("Unable to get device binding for the green LED!");
		return;
	}

	ret = gpio_pin_configure(green_dev, GREEN_PIN, GPIO_OUTPUT_ACTIVE | GREEN_FLAGS);
	if (ret < 0) {
		LOG_ERR("Unable to configure GPIO pin for the green LED!");
		return;
	}

	/* Configure the blue LED */
	blue_dev = device_get_binding(BLUE_LED);
	if (blue_dev == NULL) {
		LOG_ERR("Unable to get device binding for the blue LED!");
		return;
	}

	ret = gpio_pin_configure(blue_dev, BLUE_PIN, GPIO_OUTPUT_ACTIVE | BLUE_FLAGS);
	if (ret < 0) {
		LOG_ERR("Unable to configure GPIO pin for the blue LED!");
		return;
	}

	while (1) {
		switch (current_color) {
		case red:
			/* Enable just the red LED */
			gpio_pin_set(red_dev, RED_PIN, false);
			gpio_pin_set(green_dev, GREEN_PIN, true);
			gpio_pin_set(blue_dev, BLUE_PIN, true);
			current_color = green;
			break;
		case green:
			/* Enable just the green LED */
			gpio_pin_set(red_dev, RED_PIN, true);
			gpio_pin_set(green_dev, GREEN_PIN, false);
			gpio_pin_set(blue_dev, BLUE_PIN, true);
			current_color = blue;
			break;
		case blue:
			/* Enable just the blue LED */
			gpio_pin_set(red_dev, RED_PIN, true);
			gpio_pin_set(green_dev, GREEN_PIN, true);
			gpio_pin_set(blue_dev, BLUE_PIN, false);
			current_color = magenta;
			break;
		case magenta:
			/* Enable the blue and red LEDs */
			gpio_pin_set(red_dev, RED_PIN, false);
			gpio_pin_set(green_dev, GREEN_PIN, true);
			gpio_pin_set(blue_dev, BLUE_PIN, false);
			current_color = yellow;
			break;
		case yellow:
			/* Enable the red and green LEDs */
			gpio_pin_set(red_dev, RED_PIN, false);
			gpio_pin_set(green_dev, GREEN_PIN, false);
			gpio_pin_set(blue_dev, BLUE_PIN, true);
			current_color = cyan;
			break;
		case cyan:
			/* Enable the green and blue LEDs */
			gpio_pin_set(red_dev, RED_PIN, true);
			gpio_pin_set(green_dev, GREEN_PIN, false);
			gpio_pin_set(blue_dev, BLUE_PIN, false);
			current_color = white;
			break;
		case white:
			/* Enable all of the LEDs */
			gpio_pin_set(red_dev, RED_PIN, false);
			gpio_pin_set(green_dev, GREEN_PIN, false);
			gpio_pin_set(blue_dev, BLUE_PIN, false);
			current_color = red;
			break;
		default:
			/* This should never happen */
			LOG_ERR("Invalid color");
			return;
		}

		/* Sleep for the specified time */
		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}
}
