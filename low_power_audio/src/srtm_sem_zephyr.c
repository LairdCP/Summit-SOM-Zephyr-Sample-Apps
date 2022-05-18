/*
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include "srtm_sem.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

srtm_sem_t SRTM_Sem_Create(uint32_t maxCount, uint32_t initCount)
{
	int ret;

	struct k_sem *sem = k_malloc(sizeof(struct k_sem));
	assert(NULL != sem);

	ret = k_sem_init(sem, initCount, maxCount);
	assert(ret == 0);

	return sem;
}

void SRTM_Sem_Destroy(srtm_sem_t sem)
{
	assert(sem);

	k_free(sem);
}

srtm_status_t SRTM_Sem_Post(srtm_sem_t sem)
{
	k_sem_give((struct k_sem *)sem);

	return SRTM_Status_Success;
}

srtm_status_t SRTM_Sem_Wait(srtm_sem_t sem, uint32_t timeout)
{
	k_timeout_t wait_timeout;
	int ret = 0;

	if (__get_IPSR() != 0U || timeout == SRTM_NO_WAIT) {
		wait_timeout = K_NO_WAIT;
	} else if (timeout == SRTM_WAIT_FOR_EVER) {
		wait_timeout = K_FOREVER;
	} else {
		wait_timeout = K_MSEC(timeout);
	}

	ret = k_sem_take((struct k_sem *)sem, wait_timeout);

	switch (ret) {
	case 0:
		return SRTM_Status_Success;
	case -EAGAIN:
		return SRTM_Status_Timeout;
	default:
	case -EBUSY:
		return SRTM_Status_Error;
	}
}
