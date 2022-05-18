/*
 * Copyright 2020 NXP
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>
#include "lpm.h"

/* If block envent exists, then do not allow M7 to enter low power mode. */
static uint32_t block_event_count = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

void lpm_increase_block_sleep_count(void)
{
	block_event_count++;
}

void lpm_decrease_block_sleep_count(void)
{
	block_event_count--;
}

bool lpm_allow_sleep(void)
{
	if (block_event_count) {
		return false;
	} else {
		return true;
	}
}
