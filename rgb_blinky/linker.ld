/*
 * Copyright (c) 2022, Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <autoconf.h>
#include <devicetree.h>

MEMORY
{
#if defined(CONFIG_CODE_DDR)
	DDR  (wx) : ORIGIN = 0x80400000, LENGTH = 0x00C00000
#else
	DDR  (wx) : ORIGIN = 0x80000000, LENGTH = 0x01000000
#endif
}

#include <arch/arm/aarch32/cortex_m/scripts/linker.ld>

SECTIONS
{

#include <linker/rel-sections.ld>
#if IS_ENABLED(CONFIG_RPMSG_LITE_RESOURCE_TABLE)

	SECTION_PROLOGUE(.resource_table,, SUBALIGN(4))
	{
		KEEP(*(.resource_table*))
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif
}
