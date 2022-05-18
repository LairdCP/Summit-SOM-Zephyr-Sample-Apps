/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOW_POWER_WAKEUP_H
#define LOW_POWER_WAKEUP_H

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>

#include <drivers/ipm.h>
#include <drivers/clock_control.h>

#include <shell/shell.h>
#include <shell/shell_uart.h>

#include <version.h>

#include <device.h>
#include "lpm.h"

#include "fsl_rdc.h"
#include "fsl_gpc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define M7_DOMAIN_ID (1U)
#define A53_DOMAIN_ID (0U)

/* Using SRC_GPR10 register to sync the tasks status with A core */
#define ServiceFlagAddr SRC->GPR10
/*
 * The flags, ServiceBusy and ServiceIdle, shows if the service task is running
 * or not. If the task is runing, A core should not put DDR in self-refresh mode
 * after A core enters supsend.
 */
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)

#define MU_CHANNEL 1U
#define POLLING_TIMER_PERIOD_MS 50

/*******************************************************************************
 * Types
 ******************************************************************************/

/* LPM state of M7 core */
typedef enum {
	LPM_M7_STATE_RUN,
	LPM_M7_STATE_WAIT,
	LPM_M7_STATE_STOP,
} lpm_power_status_m7_t;

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/

void wakeup_a_core();

#endif // LOW_POWER_WAKEUP_H
