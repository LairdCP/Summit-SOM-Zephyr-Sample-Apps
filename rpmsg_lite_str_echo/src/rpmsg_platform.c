/*
 * Copyright 2019-2020 NXP
 * Copyright (c) 2022 Laird Connectivity
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "rpmsg_platform.h"
#include "rpmsg_env.h"

#include "fsl_device_registers.h"
#include "fsl_mu.h"
#include "irq.h"
#include <drivers/ipm.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(rpmsg_platform, LOG_LEVEL_DBG);

#if defined(RL_USE_ENVIRONMENT_CONTEXT) && (RL_USE_ENVIRONMENT_CONTEXT == 1)
#error "This RPMsg-Lite port requires RL_USE_ENVIRONMENT_CONTEXT set to 0"
#endif

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

static int32_t isr_counter = 0;
static int32_t disable_counter = 0;
static void *platform_lock;
static const struct device *ipm_handle;

static void platform_global_isr_disable(void)
{
	__asm volatile("cpsid i");
}

static void platform_global_isr_enable(void)
{
	__asm volatile("cpsie i");
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
	/* Register ISR to environment layer */
	env_register_isr(vector_id, isr_data);

	/* Prepare the MU Hardware, enable channel 1 interrupt */
	env_lock_mutex(platform_lock);

	RL_ASSERT(0 <= isr_counter);
	if (isr_counter == 0) {
		ipm_set_enabled(ipm_handle, 1);
	}
	isr_counter++;

	env_unlock_mutex(platform_lock);

	return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
	/* Prepare the MU Hardware */
	env_lock_mutex(platform_lock);

	RL_ASSERT(0 < isr_counter);
	isr_counter--;
	if (isr_counter == 0) {
		ipm_set_enabled(ipm_handle, 0);
	}

	/* Unregister ISR from environment layer */
	env_unregister_isr(vector_id);

	env_unlock_mutex(platform_lock);

	return 0;
}

void platform_notify(uint32_t vector_id)
{
	/* As Linux suggests, use MU->Data Channel 1 as communication channel */
	uint32_t msg = (uint32_t)(vector_id << 16);
	int ret = 0;

	env_lock_mutex(platform_lock);

	ret = ipm_send(ipm_handle, 0, RPMSG_MU_CHANNEL, &msg, 1);
	if (ret != 0) {
		LOG_WRN("ipm_send failed! ret: %d", ret);
	}

	env_unlock_mutex(platform_lock);
}

/*
 * MU Interrrupt RPMsg handler
 */
void MU1_M7_IRQHandler(const struct device *dev, void *context, uint32_t id, volatile void *data)
{
	uint32_t *data32 = (uint32_t *)data;

	/* Call the environment ISR */
	env_isr(data32[0] >> 16);
}

/**
 * platform_time_delay
 *
 * @param num_msec Delay time in ms.
 */
void platform_time_delay(uint32_t num_msec)
{
	k_sleep(K_MSEC(num_msec));
}

/**
 * platform_in_isr
 *
 * Return whether CPU is processing IRQ
 *
 * @return True for IRQ, false otherwise.
 *
 */
int32_t platform_in_isr(void)
{
	return (((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0UL) ? 1 : 0);
}

/**
 * platform_interrupt_enable
 *
 * Enable peripheral-related interrupt
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_enable(uint32_t vector_id)
{
	RL_ASSERT(0 < disable_counter);

	platform_global_isr_disable();
	disable_counter--;

	if (disable_counter == 0) {
		irq_enable(MU1_M7_IRQn);
	}
	platform_global_isr_enable();
	return ((int32_t)vector_id);
}

/**
 * platform_interrupt_disable
 *
 * Disable peripheral-related interrupt.
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_disable(uint32_t vector_id)
{
	RL_ASSERT(0 <= disable_counter);

	platform_global_isr_disable();
	/* virtqueues use the same NVIC vector
	   if counter is set - the interrupts are disabled */
	if (disable_counter == 0) {
		irq_disable(MU1_M7_IRQn);
	}
	disable_counter++;
	platform_global_isr_enable();
	return ((int32_t)vector_id);
}

/**
 * platform_map_mem_region
 *
 * Dummy implementation
 *
 */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
}

/**
 * platform_cache_all_flush_invalidate
 *
 * Dummy implementation
 *
 */
void platform_cache_all_flush_invalidate(void)
{
}

/**
 * platform_cache_disable
 *
 * Dummy implementation
 *
 */
void platform_cache_disable(void)
{
}

/**
 * platform_vatopa
 *
 * Dummy implementation
 *
 */
uint32_t platform_vatopa(void *addr)
{
	return ((uint32_t)(char *)addr);
}

/**
 * platform_patova
 *
 * Dummy implementation
 *
 */
void *platform_patova(uint32_t addr)
{
	return ((void *)(char *)addr);
}

/**
 * platform_init
 *
 * platform/environment init
 */
int32_t platform_init(void)
{
	/* setup IPM */
	ipm_handle = device_get_binding(CONFIG_RPMSG_LITE_IPC_DEV_NAME);
	if (!ipm_handle) {
		LOG_ERR("%s: Failed to find ipm device", __func__);
		return -1;
	}
	if (!device_is_ready(ipm_handle)) {
		while (1) {
		}
	}

	ipm_register_callback(ipm_handle, MU1_M7_IRQHandler, NULL);

	/* Create lock used in multi-instanced RPMsg */
	if (0 != env_create_mutex(&platform_lock, 1)) {
		return -1;
	}

	return 0;
}

/**
 * platform_deinit
 *
 * platform/environment deinit process
 */
int32_t platform_deinit(void)
{
	/* Delete lock used in multi-instanced RPMsg */
	env_delete_mutex(platform_lock);
	platform_lock = ((void *)0);
	return 0;
}
