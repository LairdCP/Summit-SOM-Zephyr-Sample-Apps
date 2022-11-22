/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2015 Xilinx, Inc. All rights reserved.
 * Copyright 2020 NXP.
 * All rights reserved.
 * Copyright (c) 2022 Laird Connectivity
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This file provides the default empty resource table data structure
 * placed in the .resource_table section of the ELF file. This facilitates
 * basic support of remoteproc firmware loading from the Linux kernel.
 *
 * The .resource_table section has to be present in the ELF file in order
 * for the remoteproc ELF parser to accept the binary.
 *
 * See other multicore examples such as those utilizing rpmsg for a examples
 * of non-empty resource table.
 *
 */

#include <stdint.h>
#include <string.h>

#define VDEV0_VRING_BASE (0x55000000U)
#define RESOURCE_TABLE_OFFSET (0xFF000)

/* Place resource table in special ELF section */
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".resource_table")))
#elif defined(__ICCARM__)
#pragma location = ".resource_table"
#else
#error Compiler not supported!
#endif

const uint32_t resource_table[] = {
	/* Version */
	1,

	/* Number of table entries - resource table empty */
	0,

	/* reserved fields */
	0, 0
};

void copy_resource_table(void)
{
	/*
	* Resource table should be copied to VDEV0_VRING_BASE + RESOURCE_TABLE_OFFSET.
	* VDEV0_VRING_BASE is temperorily kept for backward compatibility, will be removed in future
	* release
	*/
	memcpy((void *)VDEV0_VRING_BASE, &resource_table, sizeof(resource_table));
	memcpy((void *)(VDEV0_VRING_BASE + RESOURCE_TABLE_OFFSET), &resource_table, sizeof(resource_table));
}
