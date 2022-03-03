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
#define I2C_RELEASE_SDA_GPIO   GPIO5
#define I2C_RELEASE_SDA_PIN    19U
#define I2C_RELEASE_SCL_GPIO   GPIO5
#define I2C_RELEASE_SCL_PIN    18U
#define I2C_RELEASE_BUS_COUNT  100U
#define RDC_DISABLE_M7_ACCESS  0xF3
#define DOMAIN_ID   (1U)
static LPM_POWER_STATUS_M7 m7_lpm_state = LPM_M7_STATE_RUN;
/* Using SRC_GPR10 register to sync the tasks status with A core */
#define ServiceFlagAddr SRC->GPR10
/* The flags,ServiceBusy and ServiceIdle, shows if the service task is running
 * or not. If the task is runing, A core should not put DDR in self-refresh mode
 * after A core enters supsend.
 */
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)
#define SLEEP_TIME_MS	1
#define DEBUG_UART_DEVICE       DT_NODELABEL(uart4)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern volatile app_srtm_state_t srtmState;

/* Globals */
static struct k_thread app_thread;
K_THREAD_STACK_DEFINE(app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE);

/***** CODE *****/

void BOARD_PeripheralRdcSetting(void)
{
    rdc_domain_assignment_t assignment = {0};
    rdc_periph_access_config_t periphConfig;

    assignment.domainId = DOMAIN_ID;

    /* Only configure the RDC if the RDC peripheral write access is allowed. */
    if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC,
                                          assignment.domainId)) != 0U)
    {
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_PERIPH,
                                      &assignment);
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_BURST,
                                      &assignment);
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_SPBA2,
                                      &assignment);

        RDC_GetDefaultPeriphAccessConfig(&periphConfig);
        /* Do not allow the A53 domain(domain0) to access the following
         * peripherals
         */
        periphConfig.policy = RDC_DISABLE_A53_ACCESS;
        periphConfig.periph = kRDC_Periph_SAI3;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
        periphConfig.periph = kRDC_Periph_UART4;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
        periphConfig.periph = kRDC_Periph_GPT1;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
        periphConfig.periph = kRDC_Periph_SDMA3;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
        periphConfig.periph = kRDC_Periph_I2C3;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    }
}

void SAI_SetPinMux(void)
{
    IOMUXC_SetPinMux(IOMUXC_SAI3_MCLK_AUDIOMIX_SAI3_MCLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_MCLK_AUDIOMIX_SAI3_MCLK, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(3U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXC_AUDIOMIX_SAI3_TX_BCLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXC_AUDIOMIX_SAI3_TX_BCLK, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(3U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXD_AUDIOMIX_SAI3_TX_DATA0, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXD_AUDIOMIX_SAI3_TX_DATA0, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(3U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_SAI3_TXFS_AUDIOMIX_SAI3_TX_SYNC, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI3_TXFS_AUDIOMIX_SAI3_TX_SYNC, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(3U) |
                        IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
}

void LPM_MCORE_ChangeM7Clock(LPM_M7_CLOCK_SPEED target)
{
    /* Change CCM Root to change M7 clock*/
    switch (target)
    {
        case LPM_M7_LOW_FREQ:
            if (CLOCK_GetRootMux(kCLOCK_RootM7) != kCLOCK_M7RootmuxOsc24M)
            {
                CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxOsc24M);
                CLOCK_SetRootDivider(kCLOCK_RootM7, 1U, 1U);
            }
            break;
        case LPM_M7_HIGH_FREQ:
            if (CLOCK_GetRootMux(kCLOCK_RootM7) != kCLOCK_M7RootmuxSysPll1)
            {
                CLOCK_SetRootDivider(kCLOCK_RootM7, 1U, 1U);
                /* switch cortex-m7 to SYSTEM PLL1 */
                CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1);
            }
            break;
        default:
            break;
    }
}

void LPM_MCORE_SetPowerStatus(GPC_Type *base,
                              LPM_POWER_STATUS_M7 targetPowerMode)
{
    gpc_lpm_config_t config;
    config.enCpuClk              = false;
    config.enFastWakeUp          = false;
    config.enDsmMask             = false;
    config.enWfiMask             = false;
    config.enVirtualPGCPowerdown = true;
    config.enVirtualPGCPowerup   = true;
    switch (targetPowerMode)
    {
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

void PreSleepProcessing(void)
{
    APP_SRTM_Suspend();
    UART_Deinit((UART_Type *)DT_REG_ADDR(DEBUG_UART_DEVICE));
}

void PostSleepProcessing(void)
{
	uart_config_t uart_config;
	uint32_t clock_freq;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;

    APP_SRTM_Resume();

    /* Re-initialize the UART */
    clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DEBUG_UART_DEVICE));
    clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DEBUG_UART_DEVICE,
                                                          name);
	if (clock_control_get_rate(clock_dev, clock_subsys, &clock_freq) != 0) {
		clock_freq = 0;
	}

	UART_GetDefaultConfig(&uart_config);
	uart_config.enableTx = true;
	uart_config.enableRx = true;
	uart_config.baudRate_Bps = DT_PROP(DEBUG_UART_DEVICE, current_speed);

	UART_Init((UART_Type *)DT_REG_ADDR(DEBUG_UART_DEVICE), &uart_config,
              clock_freq);
}

void ShowMCoreStatus(void)
{
    if (m7_lpm_state == LPM_M7_STATE_STOP)
    {
        LOG_INF("M7 core entering STOP mode");
    }
    else if (m7_lpm_state == LPM_M7_STATE_RUN)
    {
        LOG_INF("M7 core entering RUN mode");
    }
    else
    {
        ; /* For MISRA C-2012 rule 15.7. */
    }
}

void UpdateTargetPowerStatus(void)
{
    /*
     * The m7_lpm_state merely indicates what the power state the M core
     * finally should be. In this demo, if there is no audio playback, M core
     * will be set to STOP mode finally.
     */
    LPM_POWER_STATUS_M7 m7_target_lpm;

    if (APP_SRTM_ServiceIdle())
    {
        m7_target_lpm = LPM_M7_STATE_STOP;
    }
    else
    {
        m7_target_lpm = LPM_M7_STATE_RUN;
    }

    if (m7_target_lpm != m7_lpm_state)
    {
        m7_lpm_state = m7_target_lpm;
        ShowMCoreStatus();
    }
}

void LPM_Sleep()
{
    uint32_t irqMask;
    uint64_t counter = 0;
    uint32_t timeoutTicks;
    uint32_t timeoutMilliSec = 10;   // 10ms

    while (true) {
        irqMask = DisableGlobalIRQ();

        UpdateTargetPowerStatus();

        timeoutTicks = LPM_EnterTicklessIdle(timeoutMilliSec, &counter);
        if (timeoutTicks) {
            if (LPM_AllowSleep()) {
                LPM_MCORE_ChangeM7Clock(LPM_M7_LOW_FREQ);
                LPM_MCORE_SetPowerStatus(GPC, LPM_M7_STATE_STOP);
                PreSleepProcessing();
                ServiceFlagAddr = ServiceIdle;

                __DSB();
                __ISB();
                __WFI();
                ServiceFlagAddr = ServiceBusy;
                PostSleepProcessing();

                LPM_MCORE_ChangeM7Clock(LPM_M7_HIGH_FREQ);
                LPM_MCORE_SetPowerStatus(GPC, LPM_M7_STATE_RUN);
            }
            else
            {
                __DSB();
                __ISB();
                __WFI();
            }
        }
        LPM_ExitTicklessIdle(timeoutTicks, counter);

        EnableGlobalIRQ(irqMask);

        k_sleep(K_SECONDS(1));
    }
}

void app_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Treat M7 as busy status by default.*/
    ServiceFlagAddr = ServiceBusy;

    /*
     * Wait For A53 Side to Become Ready
     */
    LOG_INF("Wait for Linux kernel to boot up and create the link between M \
            core and A core");
    while (srtmState != APP_SRTM_StateLinkedUp)
        ;
    LOG_INF("RPMsg channel created between M core and A core");

    LOG_INF("Main thread is now running");
    while (true)
    {
        LPM_Sleep();
    }
}

void main(void)
{
    uint32_t i = 0;

    BOARD_PeripheralRdcSetting();

    /* Set SAI source to AUDIO PLL1 393216000HZ*/
    CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1);
    /* Set root clock to 393216000HZ / 32 = 12.288MHz */
    CLOCK_SetRootDivider(kCLOCK_RootSai3, 1U, 32U);
    /* Set I2C source to SysPLL1 Div5 160MHZ */
    CLOCK_SetRootMux(kCLOCK_RootI2c3, kCLOCK_I2cRootmuxSysPll1Div5);
    /* Set root clock to 160MHZ / 10 = 16MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootI2c3, 1U, 10U);
    /* Set GPT source to Osc24 MHZ */
    CLOCK_SetRootMux(kCLOCK_RootGpt1, kCLOCK_GptRootmuxOsc24M);
    CLOCK_SetRootDivider(kCLOCK_RootGpt1, 1U, 1U);
    /* Enable the Audio clock on M7 side to make sure the AUDIOMIX domain clock
     * on after A core enters suspend
     */
    CLOCK_EnableClock(kCLOCK_Audio);
    /* SAI bit clock source */
    AUDIOMIX_AttachClk(AUDIOMIX, kAUDIOMIX_Attach_SAI3_MCLK1_To_SAI3_ROOT);

    SAI_SetPinMux();

    /*
     * In order to wakeup M7 from LPM, all PLLCTRLs need to be set to
     * "NeededRun"
     */
    for (i = 0; i != 39; i++)
    {
        CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
    }

    LOG_INF("LOW POWER AUDIO TASK");
    LOG_INF("Build Time: %s--%s", __DATE__, __TIME__);

    vPortSetupTimerInterrupt();

    APP_SRTM_Init();

    k_thread_create(&app_thread, app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE,
            app_task, NULL, NULL, NULL,
            -1, K_USER, K_MSEC(0));
}
