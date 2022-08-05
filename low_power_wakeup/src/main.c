/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "low_power_wakeup.h"
#include <device.h>
#include <pm/pm.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Definitions */
#define PM_MIN_TICKS 2
#define MAX_NUM_PLLS 39

/* Globals */
static const struct device *ipm_handle;
volatile bool wakeup_a_core_shell_flag = false;

extern void copy_resource_table();

/* Code */

/**
 * @brief Perform peripheral access configuration via the RDC.
 *
 */
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

/**
 * @brief Set the target power mode for the M7 core
 *
 * @param target_power_mode	Desired power mode for the M7 core
 */
void lpm_set_power_status(lpm_power_status_m7_t target_power_mode)
{
	gpc_lpm_config_t config;
	config.enCpuClk = false;
	config.enFastWakeUp = false;
	config.enDsmMask = false;
	config.enWfiMask = false;
	config.enVirtualPGCPowerdown = true;
	config.enVirtualPGCPowerup = true;
	switch (target_power_mode) {
	case LPM_M7_STATE_RUN:
		GPC->LPCR_M7 = GPC->LPCR_M7 & (~GPC_LPCR_M7_LPM0_MASK);
		break;
	case LPM_M7_STATE_WAIT:
		GPC_EnterWaitMode(GPC, &config);
		break;
	case LPM_M7_STATE_STOP:
		GPC_EnterStopMode(GPC, &config);
		break;
	default:
		break;
	}
}

/**
 * @brief Logic run before entering the LPM stop mode
 *
 */
void pre_sleep_processing(void)
{
}

/**
 * @brief Logic run after exiting the LPM stop mode
 *
 */
void post_sleep_processing(void)
{
}

/**
 * @brief Send a signal via the MU peripheral to wake up the A core.
 *
 */
void wakeup_a_core()
{
	int ret = 0;
	uint32_t msg = 2;

	LOG_INF("Sending signal to wake up the A core");
	ret = ipm_send(ipm_handle, 0, MU_CHANNEL, &msg, 1);
	if (ret != 0) {
		LOG_WRN("ipm_send failed! ret: %d", ret);
	}
}

/**
 * @brief Shell 'demo' command that prints the current board configuration.
 *
 * @param shell	Pointer to shell struct
 * @param argc	Number of arguments received
 * @param argv	Pointer to string array of arguments received
 *
 * @return	0 for success, otherwise a numerical error code
 */
static int cmd_demo_board(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, CONFIG_BOARD);

	return 0;
}

/**
 * @brief Shell 'version' command that prints the Zephyr kernel version.
 *
 * @param shell	Pointer to shell struct
 * @param argc	Number of arguments received
 * @param argv	Pointer to string array of arguments received
 *
 * @return	0 for success, otherwise a numerical error code
 */
static int cmd_version(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Zephyr version %s", KERNEL_VERSION_STRING);

	return 0;
}

/**
 * @brief Shell 'wakeup' command that triggers a signal to wake up the A core.
 *
 * @param shell	Pointer to shell struct
 * @param argc	Number of arguments received
 * @param argv	Pointer to string array of arguments received
 * @param data	Pointer to received data
 * 
 * @return	0 for success, otherwise a numerical error code
 */
static int cmd_wakeup_a_core(const struct shell *shell, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ARG_UNUSED(data);

	lpm_increase_block_sleep_count();

	wakeup_a_core_shell_flag = true;

	lpm_decrease_block_sleep_count();

	return 0;
}

/* Configure available shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo,
			       SHELL_CMD(board, NULL, "Show board name command.", cmd_demo_board),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);

SHELL_CMD_ARG_REGISTER(wakeup, NULL, "Wakeup the A core", cmd_wakeup_a_core, 1, 0);

/**
 * @brief Polling timer timeout function which handles checking the state of the
 * wakeup_a_core_shell_flag, and then waking up the A core, if necessary.
 *
 * @param timer	Pointer to timer struct
 */
static void polling_timeout(struct k_timer *timer)
{
	bool issue_wakeup = false;

	ARG_UNUSED(timer);

	if (wakeup_a_core_shell_flag) {
		wakeup_a_core_shell_flag = false;
		issue_wakeup |= true;
	}

	if (issue_wakeup) {
		wakeup_a_core();
	}
}

/* Define polling timer */
K_TIMER_DEFINE(polling_timer, polling_timeout, NULL);

/**
 * @brief Application-specific override of the Power Management set state function
 *
 * @param info	Info on the requested next state
 */
void pm_power_state_set(struct pm_state_info info)
{
	/* Extract target GPC mode from the substate ID */
	uint32_t target_mode = (uint32_t)info.substate_id;
	/* Read current GPC mode */
	uint32_t current_mode = GPC_GetLpmMode(GPC);

	irq_unlock(0);

	if (target_mode == current_mode) {
		__DSB();
		__ISB();
		__WFI();
		return;
	}

	switch (target_mode) {
	case kGPC_RunMode:
		lpm_set_power_status(LPM_M7_STATE_RUN);
		break;
	case kGPC_WaitMode:
		lpm_set_power_status(LPM_M7_STATE_WAIT);
		pre_sleep_processing();
		ServiceFlagAddr = ServiceIdle;

		__DSB();
		__ISB();
		__WFI();

		ServiceFlagAddr = ServiceBusy;
		post_sleep_processing();
		lpm_set_power_status(LPM_M7_STATE_RUN);
		break;
	case kGPC_StopMode:
		lpm_set_power_status(LPM_M7_STATE_STOP);
		pre_sleep_processing();
		ServiceFlagAddr = ServiceIdle;

		__DSB();
		__ISB();
		__WFI();

		ServiceFlagAddr = ServiceBusy;
		post_sleep_processing();
		lpm_set_power_status(LPM_M7_STATE_RUN);
		break;
	default:
		LOG_ERR("Unknown power state requested");
		return;
	}
}

/**
 * @brief Application-specific override of the Power Management power state exit handler function
 *
 * @param info	Info on the state that is currently being exited
 */
void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	ARG_UNUSED(info);
}

/**
 * @brief Application-specific Power Management state management policy
 *
 * @param ticks			The number of remaining ticks before the system wakes
 * @return struct pm_state_info	Next power state
 */
struct pm_state_info pm_policy_next_state(int32_t ticks)
{
	/*
	 * If the available PM_MIN_TICKS is less than 2 or sleep isn't allowed, then just stay in
	 * the 'active' state. Otherwise, move to the 'wait' state.
	 */
	if (ticks >= PM_MIN_TICKS && lpm_allow_sleep()) {
		return (struct pm_state_info){ PM_STATE_SUSPEND_TO_IDLE, kGPC_WaitMode, 0 };
	} else {
		return (struct pm_state_info){ PM_STATE_ACTIVE, kGPC_RunMode, 0 };
	}
}

/**
 * @brief Main function entry point
 *
 */
void main(void)
{
	uint32_t i = 0;

	/* Block LPM wait mode until initial setup is complete */
	lpm_increase_block_sleep_count();

	/* Use RDC to configure peripheral access */
	rdc_configure_peripheral_access();

	copy_resource_table();

	/*
	 * In order to wakeup M7 from LPM, all PLLCTRLs need to be set to
	 * "NeededRun"
	 */
	for (i = 0; i != MAX_NUM_PLLS; i++) {
		CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
	}

	/*
	 * Set the ServiceFlag to ServiceBusy so that the A core does not attempt to stop the M7
	 * core when it enters into the DSM state.
	 */
	ServiceFlagAddr = ServiceBusy;

	LOG_INF("LOW POWER WAKEUP EXAMPLE");

	/* Setup the IPM (MU) peripheral */
	ipm_handle = device_get_binding(CONFIG_RPMSG_LITE_IPC_DEV_NAME);
	if (!ipm_handle) {
		LOG_ERR("Failed to find ipm device");
		return;
	}
	while (!device_is_ready(ipm_handle)) {
		LOG_ERR("MU is not ready");
		k_sleep(K_SECONDS(1));
	}

	/* Clear the terminal */
	shell_execute_cmd(shell_backend_uart_get_ptr(), "clear");

	/*
	 * Start polling timer.
	 */
	k_timer_start(&polling_timer, K_MSEC(POLLING_TIMER_PERIOD_MS),
		      K_MSEC(POLLING_TIMER_PERIOD_MS));

	/* Unblock LPM wait mode and sleep the main thread */
	lpm_decrease_block_sleep_count();
	k_sleep(K_FOREVER);
}
