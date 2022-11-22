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

LOG_MODULE_REGISTER(buttons, LOG_LEVEL_INF);

/*
 * Get button configuration from the devicetree volume button aliases.
 * This is mandatory.
 */
#define VOLUME_DOWN_BTN_NODE DT_ALIAS(volumedownbtn)
#define VOLUME_UP_BTN_NODE DT_ALIAS(volumeupbtn)
#if !DT_NODE_HAS_STATUS(VOLUME_DOWN_BTN_NODE, okay)
#error "Unsupported board: volumedownbtn devicetree alias is not defined"
#endif
#if !DT_NODE_HAS_STATUS(VOLUME_UP_BTN_NODE, okay)
#error "Unsupported board: volumeupbtn devicetree alias is not defined"
#endif

static const struct gpio_dt_spec volume_down_btn =
	GPIO_DT_SPEC_GET_OR(VOLUME_DOWN_BTN_NODE, gpios, { 0 });
static const struct gpio_dt_spec volume_up_btn =
	GPIO_DT_SPEC_GET_OR(VOLUME_UP_BTN_NODE, gpios, { 0 });
static struct gpio_callback volume_down_btn_cb_data;
static struct gpio_callback volume_up_btn_cb_data;

extern void copy_resource_table();

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
		periph_config.periph = kRDC_Periph_I2C3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_SAI3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
	}
}

void volume_down_btn_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("Volume down button pressed at %" PRIu32, k_cycle_get_32());
}

void volume_up_btn_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("Volume up button pressed at %" PRIu32, k_cycle_get_32());
}

void main(void)
{
	int ret;

	LOG_INF("*** Buttons Demo Start ***");

	rdc_configure_peripheral_access();

	copy_resource_table();

	if (!device_is_ready(volume_down_btn.port)) {
		LOG_ERR("Error: button device %s is not ready", volume_down_btn.port->name);
		return;
	}

	if (!device_is_ready(volume_up_btn.port)) {
		LOG_ERR("Error: button device %s is not ready", volume_up_btn.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&volume_down_btn, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d", ret, volume_down_btn.port->name,
			volume_down_btn.pin);
		return;
	}
	ret = gpio_pin_configure_dt(&volume_up_btn, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d", ret, volume_up_btn.port->name,
			volume_up_btn.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&volume_down_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret,
			volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	ret = gpio_pin_interrupt_configure_dt(&volume_up_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret,
			volume_up_btn.port->name, volume_up_btn.pin);
		return;
	}

	gpio_init_callback(&volume_down_btn_cb_data, volume_down_btn_pressed,
			   BIT(volume_down_btn.pin));
	ret = gpio_add_callback(volume_down_btn.port, &volume_down_btn_cb_data);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to set up callback on %s pin %d", ret,
			volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	LOG_INF("Set up button at %s pin %d", volume_down_btn.port->name, volume_down_btn.pin);

	gpio_init_callback(&volume_up_btn_cb_data, volume_up_btn_pressed, BIT(volume_up_btn.pin));
	ret = gpio_add_callback(volume_up_btn.port, &volume_up_btn_cb_data);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to set up callback on %s pin %d", ret,
			volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	LOG_INF("Set up button at %s pin %d", volume_up_btn.port->name, volume_up_btn.pin);

	LOG_INF("Press a button");
}
