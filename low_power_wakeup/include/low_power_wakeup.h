/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOW_POWER_WAKEUP_H
#define LOW_POWER_WAKEUP_H

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/clock_control.h>
#include <sys/printk.h>
#include <stdint.h>
#include <string.h>
#include <logging/log.h>

#include "fsl_rdc.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include <drivers/ipm.h>

#include <shell/shell.h>
#include <version.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <ctype.h>
#include <shell/shell_uart.h>
#include <device.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RDC_DISABLE_A53_ACCESS      0xFC
#define RDC_A53_READ_ONLY_ACCESS    0xFE
#define DOMAIN_ID                   (1U)

/* Using SRC_GPR10 register to sync the tasks status with A core */
#define ServiceFlagAddr SRC->GPR10
/*
 * The flags, ServiceBusy and ServiceIdle, shows if the service task is running
 * or not. If the task is runing, A core should not put DDR in self-refresh mode
 * after A core enters supsend.
 */
#define ServiceBusy                 (0x5555U)
#define ServiceIdle                 (0x0U)

#define DEBUG_UART_DEVICE           DT_NODELABEL(uart4)
#define VOLUME_UP_BTN_NODE	        DT_ALIAS(volumeupbtn)
#define VOLUME_DOWN_BTN_NODE	    DT_ALIAS(volumedownbtn)
#define MU_CHANNEL                  1U

/* Types */

/*! @brief Shell state definition for the low power wakeup example */
typedef enum _shell_state
{
    LPW_SHELL_STATE_UNINITIALIZED   = 0U,   /* Shell is uninitialized */
    LPW_SHELL_STATE_INITIALIZING    = 1U,   /* Shell is initializing */
    LPW_SHELL_STATE_INITIALIZED     = 2U    /* Shell is initialized */
} lpw_shell_state_t;

#endif // LOW_POWER_WAKEUP_H
