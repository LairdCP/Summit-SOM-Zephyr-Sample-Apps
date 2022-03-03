/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/*
 * Get button configuration from the devicetree volume button aliases.
 * This is mandatory.
 */
#define VOLUME_DOWN_BTN_NODE	DT_ALIAS(volumedownbtn)
#define VOLUME_UP_BTN_NODE	    DT_ALIAS(volumeupbtn)
#if !DT_NODE_HAS_STATUS(VOLUME_DOWN_BTN_NODE, okay)
#error "Unsupported board: volumedownbtn devicetree alias is not defined"
#endif
#if !DT_NODE_HAS_STATUS(VOLUME_UP_BTN_NODE, okay)
#error "Unsupported board: volumeupbtn devicetree alias is not defined"
#endif

static const struct gpio_dt_spec volume_down_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_DOWN_BTN_NODE, gpios, {0});
static const struct gpio_dt_spec volume_up_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_UP_BTN_NODE, gpios, {0});
static struct gpio_callback volume_down_btn_cb_data;
static struct gpio_callback volume_up_btn_cb_data;

void volume_down_btn_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Volume down button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void volume_up_btn_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Volume up button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void main(void)
{
	int ret;

	printk("*** Buttons Demo Start ***\n");

    if (!device_is_ready(volume_down_btn.port)) {
		printk("Error: button device %s is not ready\n",
		       volume_down_btn.port->name);
		return;
    }

    if (!device_is_ready(volume_up_btn.port)) {
		printk("Error: button device %s is not ready\n",
		       volume_up_btn.port->name);
		return;
    }

    ret = gpio_pin_configure_dt(&volume_down_btn, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
    ret = gpio_pin_configure_dt(&volume_up_btn, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, volume_up_btn.port->name, volume_up_btn.pin);
		return;
	}

    ret = gpio_pin_interrupt_configure_dt(&volume_down_btn,
                                   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
    ret = gpio_pin_interrupt_configure_dt(&volume_up_btn,
                                   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, volume_up_btn.port->name, volume_up_btn.pin);
		return;
	}
    
    gpio_init_callback(&volume_down_btn_cb_data, volume_down_btn_pressed,
                       BIT(volume_down_btn.pin));
    ret = gpio_add_callback(volume_down_btn.port, &volume_down_btn_cb_data);
	if (ret != 0) {
		printk("Error %d: failed to set up callback on %s pin %d\n",
			ret, volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	printk("Set up button at %s pin %d\n", volume_down_btn.port->name, volume_down_btn.pin);
    
    gpio_init_callback(&volume_up_btn_cb_data, volume_up_btn_pressed,
                       BIT(volume_up_btn.pin));
    ret = gpio_add_callback(volume_up_btn.port, &volume_up_btn_cb_data);
	if (ret != 0) {
		printk("Error %d: failed to set up callback on %s pin %d\n",
			ret, volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	printk("Set up button at %s pin %d\n", volume_up_btn.port->name, volume_up_btn.pin);
	
	printk("Press a button\n");
}
