/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOW_POWER_AUDIO_H
#define LOW_POWER_AUDIO_H

#include "fsl_gpc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Shared memory base for RPMsg communication. */
#define VDEV0_VRING_BASE      (0x55000000U)
#define RESOURCE_TABLE_OFFSET (0xFF000)

#define APP_PowerUpSlot (5U)
#define APP_PowerDnSlot (6U)

#define CODEC_I2C            I2C3
#define CODEC_I2C_INSTANCE   (3U)
#define CODEC_I2C_CLOCK_FREQ (16000000U)

/*
 * LPM state of M7 core
 */
typedef enum lpm_power_status_m7
{
    LPM_M7_STATE_RUN,
    LPM_M7_STATE_WAIT,
    LPM_M7_STATE_STOP,
} LPM_POWER_STATUS_M7;
/*
 * Clock Speed of M7 core
 */
typedef enum lpm_m7_clock_speed
{
    LPM_M7_HIGH_FREQ,
    LPM_M7_LOW_FREQ
} LPM_M7_CLOCK_SPEED;

#endif // LOW_POWER_AUDIO_H
