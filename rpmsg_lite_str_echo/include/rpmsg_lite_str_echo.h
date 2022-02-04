/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RPMSG_LITE_STR_ECHO_H
#define RPMSG_LITE_STR_ECHO_H

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Shared memory base for RPMsg communication. */
#define VDEV0_VRING_BASE      (0x55000000U)
#define RESOURCE_TABLE_OFFSET (0xFF000)

#define RPMSG_LITE_SHMEM_BASE         (VDEV0_VRING_BASE)
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8MP_M7_USER_LINK_ID)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel-1"
#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

#endif // RPMSG_LITE_STR_ECHO_H
