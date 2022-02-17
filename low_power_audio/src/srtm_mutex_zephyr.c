/*
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include "srtm_mutex.h"

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

srtm_mutex_t SRTM_Mutex_Create(void)
{
    int ret;

    struct k_mutex *mutex = k_malloc(sizeof(struct k_mutex));
    assert(NULL != mutex);

    ret = k_mutex_init(mutex);
    assert(ret == 0);

    return (srtm_mutex_t)mutex;
}

void SRTM_Mutex_Destroy(srtm_mutex_t mutex)
{
    assert(mutex != NULL);

    k_free(mutex);
}

srtm_status_t SRTM_Mutex_Lock(srtm_mutex_t mutex)
{
    assert(mutex != NULL);

    if (0 != k_mutex_lock((struct k_mutex *)mutex, K_FOREVER)) {
        return SRTM_Status_Error;
    }

    return SRTM_Status_Success;
}

srtm_status_t SRTM_Mutex_Unlock(srtm_mutex_t mutex)
{
    assert(mutex != NULL);

    if (0 != k_mutex_unlock((struct k_mutex *)mutex)) {
        return SRTM_Status_Error;
    }

    return SRTM_Status_Success;
}
