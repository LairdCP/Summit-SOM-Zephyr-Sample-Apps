/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdint.h>
#include <string.h>

#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rsc_table.h"
#include "rpmsg_lite_str_echo.h"

#include <logging/log.h>

#include "fsl_rdc.h"

#define M7_DOMAIN_ID (1U)
#define A53_DOMAIN_ID (0U)

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Globals */
/* Each RPMSG buffer can carry less than 512 payload */
static char app_buf[512];
static struct k_thread app_thread;
K_THREAD_STACK_DEFINE(app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE);

void rdc_configure_peripheral_access(void)
{
	rdc_domain_assignment_t assignment = { 0 };
	rdc_periph_access_config_t periph_config;

	assignment.domainId = M7_DOMAIN_ID;

	/* Only configure the RDC if the RDC peripheral write access is allowed. */
	if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, assignment.domainId)) != 0U) {
		RDC_GetDefaultPeriphAccessConfig(&periph_config);

		/* Give the A53 domain (domain0) access to the I2C3 and SAI3 peripherals. */
		periph_config.policy = RDC_ACCESS_POLICY(A53_DOMAIN_ID, kRDC_ReadWrite);
		periph_config.periph = kRDC_Periph_I2C3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_SAI3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
	}
}

void app_task(void *p1, void *p2, void *p3)
{
	volatile uint32_t remote_addr;
	struct rpmsg_lite_endpoint *volatile my_ept;
	volatile rpmsg_queue_handle my_queue;
	struct rpmsg_lite_instance *volatile my_rpmsg;
	void *rx_buf;
	uint32_t len;
	int32_t result;
	void *tx_buf;
	uint32_t size;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	copy_resource_table();

	/* Print the initial banner */
	LOG_INF("RPMSG String Echo Zephyr RTOS API Demo");

	my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID,
					  RL_NO_FLAGS);
	__ASSERT(my_rpmsg != RL_NULL, "Unable to init rpmsg lite");

	while (0 == rpmsg_lite_is_link_up(my_rpmsg))
		;

	my_queue = rpmsg_queue_create(my_rpmsg);
	my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
	(void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

	LOG_INF("Nameservice sent, ready for incoming messages...");

	for (;;) {
		/* Get RPMsg rx buffer with message */
		result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr,
						 (char **)&rx_buf, &len, RL_BLOCK);
		if (result != 0) {
			__ASSERT(false, "Failed to queue the message");
		}

		/* Copy string from RPMsg rx buffer */
		__ASSERT(len < sizeof(app_buf), "rx message is too big");
		memcpy(app_buf, rx_buf, len);
		/* End string by '\0' */
		app_buf[len] = 0;

		if ((len == 2) && (app_buf[0] == 0xd) && (app_buf[1] == 0xa)) {
			LOG_INF("Get New Line From Master Side");
		} else {
			LOG_INF("Get Message From Master Side : \"%s\" [len : %d]", log_strdup(app_buf), len);
		}

		/* Get tx buffer from RPMsg */
		tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
		__ASSERT(tx_buf, "Failed to read tx buffer");
		/* Copy string to RPMsg tx buffer */
		memcpy(tx_buf, app_buf, len);
		/* Echo back received message with nocopy send */
		result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len);
		if (result != 0) {
			__ASSERT(false, "Failed to send message");
		}
		/* Release held RPMsg rx buffer */
		result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
		if (result != 0) {
			__ASSERT(false, "Failed to free the rx buffer");
		}
	}
}

void main(void)
{
	rdc_configure_peripheral_access();

	k_thread_create(&app_thread, app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE, app_task, NULL,
			NULL, NULL, -1, K_USER, K_MSEC(0));
}
