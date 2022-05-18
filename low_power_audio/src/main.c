/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/clock_control.h>
#include <sys/printk.h>
#include <stdint.h>
#include <string.h>
#include <logging/log.h>
#include <pm/pm.h>

#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rsc_table.h"
#include "low_power_audio.h"
#include "lpm.h"
#include "fsl_audiomix.h"
#include "fsl_rdc.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_uart.h"
#include "app_srtm.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RDC_DISABLE_A53_ACCESS 0xFC
#define RDC_DISABLE_M7_ACCESS 0xF3
#define DOMAIN_ID (1U)
static lpm_power_status_m7_t m7_lpm_state = LPM_M7_STATE_RUN;
/* Using SRC_GPR10 register to sync the tasks status with A core */
#define ServiceFlagAddr SRC->GPR10
/*
 * The flags, ServiceBusy and ServiceIdle, shows if the service task is running or not. If the task
 * is running, A core should not put DDR in self-refresh mode after A core enters suspend.
 */
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)
#define SLEEP_TIME_MS 1
#define PM_MIN_TICKS 2
#define MAX_NUM_PLLS 39

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern volatile app_srtm_state_t srtm_state;

/* Globals */
static struct k_thread app_thread;
K_THREAD_STACK_DEFINE(app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE);

/***** CODE *****/

/**
 * @brief Configure RDC
 *
 */
void rdc_configure_peripheral_access(void)
{
	rdc_domain_assignment_t assignment = { 0 };
	rdc_periph_access_config_t periph_config;

	assignment.domainId = DOMAIN_ID;

	/* Only configure the RDC if the RDC peripheral write access is allowed. */
	if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, assignment.domainId)) != 0U) {
		RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_PERIPH, &assignment);
		RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_BURST, &assignment);
		RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_SPBA2, &assignment);

		RDC_GetDefaultPeriphAccessConfig(&periph_config);
		/* Do not allow the A53 domain (domain0) to access the following peripherals */
		periph_config.policy = RDC_DISABLE_A53_ACCESS;
		periph_config.periph = kRDC_Periph_SAI3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_UART4;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_GPT1;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_SDMA3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
		periph_config.periph = kRDC_Periph_I2C3;
		RDC_SetPeriphAccessConfig(RDC, &periph_config);
	}
}

/**
 * @brief Configure pin mux for the SAI3 peripheral
 *
 */
void set_sai_pin_mux(void)
{
	IOMUXC_SetPinMux(IOMUXC_SAI3_MCLK_AUDIOMIX_SAI3_MCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_SAI3_MCLK_AUDIOMIX_SAI3_MCLK,
			    IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
	IOMUXC_SetPinMux(IOMUXC_SAI3_TXC_AUDIOMIX_SAI3_TX_BCLK, 0U);
	IOMUXC_SetPinConfig(IOMUXC_SAI3_TXC_AUDIOMIX_SAI3_TX_BCLK,
			    IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
	IOMUXC_SetPinMux(IOMUXC_SAI3_TXD_AUDIOMIX_SAI3_TX_DATA0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_SAI3_TXD_AUDIOMIX_SAI3_TX_DATA0,
			    IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
	IOMUXC_SetPinMux(IOMUXC_SAI3_TXFS_AUDIOMIX_SAI3_TX_SYNC, 0U);
	IOMUXC_SetPinConfig(IOMUXC_SAI3_TXFS_AUDIOMIX_SAI3_TX_SYNC,
			    IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
				    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
}

/**
 * @brief Set the M7 clock source between two supported modes: LOW - 24 MHz oscillator, HIGH -
 * SYSPLL1.
 *
 * @param target	Target clock speed
 */
void lpm_change_m7_clock(lpm_m7_clock_speed_t target)
{
	/* Change CCM Root to change M7 clock*/
	switch (target) {
	case LPM_M7_LOW_FREQ:
		if (CLOCK_GetRootMux(kCLOCK_RootM7) != kCLOCK_M7RootmuxOsc24M) {
			CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxOsc24M);
			CLOCK_SetRootDivider(kCLOCK_RootM7, 1U, 1U);
		}
		break;
	case LPM_M7_HIGH_FREQ:
		if (CLOCK_GetRootMux(kCLOCK_RootM7) != kCLOCK_M7RootmuxSysPll1) {
			CLOCK_SetRootDivider(kCLOCK_RootM7, 1U, 1U);
			/* switch cortex-m7 to SYSTEM PLL1 */
			CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1);
		}
		break;
	default:
		break;
	}
}

/**
 * @brief Set the LPM mode
 *
 * @param target_power_mode	Target LPM mode
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
	app_srtm_suspend();

	/* De-initialize the UART */
	UART_Deinit((UART_Type *)DT_REG_ADDR(DT_NODELABEL(uart4)));
}

/**
 * @brief Logic run after exiting the LPM stop mode
 *
 */
void post_sleep_processing(void)
{
	uart_config_t uart_config;
	uint32_t clock_freq;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	app_srtm_resume();

	/* Re-initialize the UART */
	clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_NODELABEL(uart4)));
	clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_NODELABEL(uart4), name);
	if (clock_control_get_rate(clock_dev, clock_subsys, &clock_freq) != 0) {
		clock_freq = 0;
	}

	UART_GetDefaultConfig(&uart_config);
	uart_config.enableTx = true;
	uart_config.enableRx = true;
	uart_config.baudRate_Bps = DT_PROP(DT_NODELABEL(uart4), current_speed);

	UART_Init((UART_Type *)DT_REG_ADDR(DT_NODELABEL(uart4)), &uart_config, clock_freq);
}

/**
 * @brief Print a message to the console about the current power mode
 *
 */
void show_m_core_status(void)
{
	switch (m7_lpm_state) {
	case LPM_M7_STATE_STOP:
		printk("M7 core entering STOP mode\n\n");
		break;
	case LPM_M7_STATE_WAIT:
		printk("M7 core entering WAIT mode\n\n");
		break;
	case LPM_M7_STATE_RUN:
		printk("M7 core entering RUN mode\n\n");
		break;
	default:
		/* Shouldn't happen */
		break;
	}
}

/**
 * @brief Update the target power mode based on the state of the SRTM service. If the mode changes,
 * print a message to the console.
 *
 */
void update_target_power_status(void)
{
	/*
	 * The m7_lpm_state merely indicates what the power state the M core finally should be. In
	 * this demo, if there is no audio playback, M core will be set to STOP mode finally.
	 */
	lpm_power_status_m7_t m7_target_lpm;

	if (app_srtm_service_idle()) {
		m7_target_lpm = LPM_M7_STATE_STOP;
	} else {
		m7_target_lpm = LPM_M7_STATE_RUN;
	}

	if (m7_target_lpm != m7_lpm_state) {
		m7_lpm_state = m7_target_lpm;
		show_m_core_status();
	}
}

/**
 * @brief Main app task function
 *
 * @param p1	Unused
 * @param p2	Unused
 * @param p3	Unused
 */
void app_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	lpm_increase_block_sleep_count();

	/* Treat M7 as busy status by default.*/
	ServiceFlagAddr = ServiceBusy;

	/*
	* Wait For A53 Side to Become Ready
	*/
	printk("Wait for Linux kernel to boot up and create the link between M core and A core\n");
	while (srtm_state != APP_SRTM_STATE_LINKED_UP)
		;
	printk("RPMsg channel created between M core and A core\n");

	printk("Main thread is now running\n");

	/* Small delay to let things settle */
	k_sleep(K_MSEC(1000));

	lpm_decrease_block_sleep_count();

	while (true) {
		k_sleep(K_FOREVER);
	}
}

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
		return;
	case kGPC_WaitMode:
		lpm_change_m7_clock(LPM_M7_LOW_FREQ);
		lpm_set_power_status(LPM_M7_STATE_WAIT);
		pre_sleep_processing();
		ServiceFlagAddr = ServiceIdle;

		__DSB();
		__ISB();
		__WFI();

		ServiceFlagAddr = ServiceBusy;
		post_sleep_processing();
		lpm_change_m7_clock(LPM_M7_HIGH_FREQ);
		lpm_set_power_status(LPM_M7_STATE_RUN);
		return;
	case kGPC_StopMode:
		lpm_change_m7_clock(LPM_M7_LOW_FREQ);
		lpm_set_power_status(LPM_M7_STATE_STOP);
		pre_sleep_processing();
		ServiceFlagAddr = ServiceIdle;

		__DSB();
		__ISB();
		__WFI();

		ServiceFlagAddr = ServiceBusy;
		post_sleep_processing();
		lpm_change_m7_clock(LPM_M7_HIGH_FREQ);
		lpm_set_power_status(LPM_M7_STATE_RUN);
		return;
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
	update_target_power_status();

	/*
	 * If the available PM_MIN_TICKS is less than 2 or sleep isn't allowed, then just stay in
	 * the 'active' state. Otherwise, move to the 'stop' state.
	 */
	if (ticks >= PM_MIN_TICKS && app_srtm_service_idle() && lpm_allow_sleep()) {
		return (struct pm_state_info){ PM_STATE_SUSPEND_TO_IDLE, kGPC_StopMode, 0 };
	} else {
		return (struct pm_state_info){ PM_STATE_ACTIVE, kGPC_RunMode, 0 };
	}
}

/**
 * @brief Main application entry point
 *
 */
void main(void)
{
	uint32_t i = 0;

	lpm_increase_block_sleep_count();

	/* Enable the CCGR gate for SysPLL1 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for SysPLL2 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for SysPLL3 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for AudioPLL1 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for AudioPLL2 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for VideoPLL1 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNeededAll);

	rdc_configure_peripheral_access();

	set_sai_pin_mux();

	/* Set SAI source to AUDIO PLL1 393216000HZ */
	CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1);
	/* Set root clock to 393216000HZ / 32 = 12.288MHz */
	CLOCK_SetRootDivider(kCLOCK_RootSai3, 1U, 32U);

	/*
	 * Enable the Audio clock on M7 side to make sure the AUDIOMIX domain clock on after A core
	 * enters suspend
	 */
	CLOCK_EnableClock(kCLOCK_Audio);
	/* SAI bit clock source */
	AUDIOMIX_AttachClk(AUDIOMIX, kAUDIOMIX_Attach_SAI3_MCLK1_To_SAI3_ROOT);

	/* In order to wakeup M7 from LPM, all PLLCTRLs need to be set to "NeededRun" */
	for (i = 0; i != MAX_NUM_PLLS; i++) {
		CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
	}

	printk("LOW POWER AUDIO TASK\n");
	printk("Build Time: %s--%s\n", __DATE__, __TIME__);

	app_srtm_init();

	k_thread_create(&app_thread, app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE, app_task, NULL,
			NULL, NULL, -1, K_USER, K_MSEC(0));

	lpm_decrease_block_sleep_count();

	while (true) {
		k_sleep(K_FOREVER);
	}
}
