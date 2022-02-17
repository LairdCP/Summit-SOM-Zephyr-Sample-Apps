/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <stdint.h>
#include <string.h>

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
#include "app_srtm.h"

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
/* The flags,ServiceBusy and ServiceIdle, shows if the service task is running or not.
 * If the task is runing, A core should not put DDR in self-refresh mode after A core enters supsend.
 */
#define ServiceBusy (0x5555U)
#define ServiceIdle (0x0U)
#define SLEEP_TIME_MS	1

/*
 * Get button configuration from the devicetree button3 alias. This is mandatory.
 */
#define VOLUME_DOWN_BTN_NODE	DT_ALIAS(volumedownbtn)
#define VOLUME_UP_BTN_NODE	    DT_ALIAS(volumeupbtn)
#if !DT_NODE_HAS_STATUS(VOLUME_DOWN_BTN_NODE, okay)
#error "Unsupported board: volumedownbtn devicetree alias is not defined"
#endif
#if !DT_NODE_HAS_STATUS(VOLUME_UP_BTN_NODE, okay)
#error "Unsupported board: volumeupbtn devicetree alias is not defined"
#endif
static const struct gpio_dt_spec volume_down_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_DOWN_BTN_NODE, gpios, {0});
static const struct gpio_dt_spec volume_up_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_UP_BTN_NODE, gpios, {0});
static struct gpio_callback volume_down_btn_cb_data;
static struct gpio_callback volume_up_btn_cb_data;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);
extern volatile app_srtm_state_t srtmState;

/* Globals */
// static char app_buf[512]; /* Each RPMSG buffer can carry less than 512 payload */
static struct k_thread app_thread;
// static struct k_thread read_buttons_thread;
K_THREAD_STACK_DEFINE(app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE);
// K_THREAD_STACK_DEFINE(read_buttons_stack, 1024);

/***** CODE *****/

void BOARD_PeripheralRdcSetting(void)
{
    rdc_domain_assignment_t assignment = {0};
    rdc_periph_access_config_t periphConfig;

    assignment.domainId = DOMAIN_ID;

    /* Only configure the RDC if the RDC peripheral write access is allowed. */
    if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, assignment.domainId)) != 0U)
    {
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_PERIPH, &assignment);
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_BURST, &assignment);
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_SDMA3_SPBA2, &assignment);

        RDC_GetDefaultPeriphAccessConfig(&periphConfig);
        /* Do not allow the A53 domain(domain0) to access the following peripherals */
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

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_I2C_ConfigurePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_I2C_ConfigurePins(void) {                       /*!< Function assigned for the core: Cortex-M7F[m7] */
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_I2C3_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SCL_I2C3_SCL, 
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PE_MASK);
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_I2C3_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SDA_I2C3_SDA, 
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PE_MASK);
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i                    = 0;
    gpio_pin_config_t pin_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};

    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_GPIO5_IO18, 0U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SCL_GPIO5_IO18, IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                                                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_GPIO5_IO19, 0U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SDA_GPIO5_IO19, IOMUXC_SW_PAD_CTL_PAD_DSE(3U) | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK |
                                                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
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
                CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1); /* switch cortex-m7 to SYSTEM PLL1 */
            }
            break;
        default:
            break;
    }
}

void LPM_MCORE_SetPowerStatus(GPC_Type *base, LPM_POWER_STATUS_M7 targetPowerMode)
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
    // DbgConsole_Deinit();
}

void PostSleepProcessing(void)
{
    APP_SRTM_Resume();
    // DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE,
    //                 BOARD_DEBUG_UART_CLK_FREQ);
}

void ShowMCoreStatus(void)
{
    if (m7_lpm_state == LPM_M7_STATE_STOP)
    {
        printk("\r\nM7 core entering STOP mode!\r\n");
    }
    else if (m7_lpm_state == LPM_M7_STATE_RUN)
    {
        printk("\r\nM7 core entering RUN mode!\r\n");
    }
    else
    {
        ; /* For MISRA C-2012 rule 15.7. */
    }
}

void UpdateTargetPowerStatus(void)
{
    /*
     * The m7_lpm_state merely indicates what the power state the M core finally should be.
     * In this demo, if there is no audio playback, M core will be set to STOP mode finally.
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

void app_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Treat M7 as busy status by default.*/
    ServiceFlagAddr = ServiceBusy;

    /*
     * Wait For A53 Side Become Ready
     */
    printk("********************************\r\n");
    printk(" Wait for Linux kernel to boot up and create the link between M core and A core.\r\n");
    printk("\r\n");
    printk("********************************\r\n");
    while (srtmState != APP_SRTM_StateLinkedUp)
        ;
    printk("The rpmsg channel created between M core and A core!\r\n");
    printk("********************************\r\n");
    printk("\r\n");

    /* Configure GPC */
    GPC_Init(GPC, APP_PowerUpSlot, APP_PowerDnSlot);
    GPC_EnableIRQ(GPC, MU1_M7_IRQn);
    GPC_EnableIRQ(GPC, GPT1_IRQn);
    while (true)
    {
        printk("\r\nMain thread is now running.\r\n");
        k_sleep(K_FOREVER);
    }

    // volatile uint32_t remote_addr;
    // struct rpmsg_lite_endpoint *volatile my_ept;
    // volatile rpmsg_queue_handle my_queue;
    // struct rpmsg_lite_instance *volatile my_rpmsg;
    // void *rx_buf;
    // uint32_t len;
    // int32_t result;
    // void *tx_buf;
    // uint32_t size;

    // ARG_UNUSED(p1);
    // ARG_UNUSED(p2);
    // ARG_UNUSED(p3);

    // copyResourceTable();

    // /* Print the initial banner */
    // printk("\r\nLaird Connectivity Low Power Audio Example...\r\n");

    // my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    // __ASSERT(my_rpmsg != RL_NULL, "Unable to init rpmsg lite");

    // while (0 == rpmsg_lite_is_link_up(my_rpmsg))
    //     ;

    // my_queue = rpmsg_queue_create(my_rpmsg);
    // my_ept   = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    // (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    // printk("\r\nNameservice sent, ready for incoming messages...\r\n");

    // for (;;)
    // {
    //     /* Get RPMsg rx buffer with message */
    //     result =
    //         rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char **)&rx_buf, &len, RL_BLOCK);
    //     if (result != 0)
    //     {
    //         __ASSERT(false, "Failed to queue the message");
    //     }

    //     /* Copy string from RPMsg rx buffer */
    //     __ASSERT(len < sizeof(app_buf), "rx message is too big");
    //     memcpy(app_buf, rx_buf, len);
    //     app_buf[len] = 0; /* End string by '\0' */

    //     if ((len == 2) && (app_buf[0] == 0xd) && (app_buf[1] == 0xa))
    //         printk("Get New Line From Master Side\r\n");
    //     else
    //         printk("Get Message From Master Side : \"%s\" [len : %d]\r\n", app_buf, len);

    //     /* Get tx buffer from RPMsg */
    //     tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
    //     __ASSERT(tx_buf, "Failed to read tx buffer");
    //     /* Copy string to RPMsg tx buffer */
    //     memcpy(tx_buf, app_buf, len);
    //     /* Echo back received message with nocopy send */
    //     result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len);
    //     if (result != 0)
    //     {
    //         __ASSERT(false, "Failed to send message");
    //     }
    //     /* Release held RPMsg rx buffer */
    //     result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
    //     if (result != 0)
    //     {
    //         __ASSERT(false, "Failed to free the rx buffer");
    //     }
    // }

}

void go_to_sleep()
{
    uint32_t irqMask;
    uint64_t counter = 0;
    uint32_t timeoutTicks;
    uint32_t timeoutMilliSec = 10000;   // 10 seconds
    // uint32_t timeoutMilliSec = (uint32_t)((uint64_t)1000 * xExpectedIdleTime / configTICK_RATE_HZ);

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
}

void volume_down_btn_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Volume down button pressed at %" PRIu32 "\n", k_cycle_get_32());

}

void volume_up_btn_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Volume up button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void read_buttons_task(void *p1, void *p2, void *p3)
{
    int ret;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("%s: start\n", __func__);

    if (!device_is_ready(volume_down_btn.port)) {
        __ASSERT(false, "Error - volume down button device is not ready");
    }

    if (!device_is_ready(volume_up_btn.port)) {
        __ASSERT(false, "Error - volume up button device is not ready");
    }

    ret = gpio_pin_configure_dt(&volume_down_btn, GPIO_INPUT);
    __ASSERT(ret == 0, "Error %d: failed to configure %s pin %d",
             ret, volume_down_btn.port->name, volume_down_btn.pin);
    ret = gpio_pin_configure_dt(&volume_up_btn, GPIO_INPUT);
    __ASSERT(ret == 0, "Error %d: failed to configure %s pin %d",
             ret, volume_up_btn.port->name, volume_up_btn.pin);

    ret = gpio_pin_interrupt_configure_dt(&volume_down_btn,
                                   GPIO_INT_EDGE_TO_ACTIVE);
    __ASSERT(ret == 0, "Error %d: failed to configure interrupt on %s pin %d",
             ret, volume_down_btn.port->name, volume_down_btn.pin);
    ret = gpio_pin_interrupt_configure_dt(&volume_up_btn,
                                   GPIO_INT_EDGE_TO_ACTIVE);
    __ASSERT(ret == 0, "Error %d: failed to configure interrupt on %s pin %d",
             ret, volume_up_btn.port->name, volume_up_btn.pin);
    
    gpio_init_callback(&volume_down_btn_cb_data, volume_down_btn_pressed,
                       BIT(volume_down_btn.pin));
    gpio_add_callback(volume_down_btn.port, &volume_down_btn_cb_data);
	printk("Set up volume down button at %s pin %d\n",
           volume_down_btn.port->name, volume_down_btn.pin);
    
    gpio_init_callback(&volume_up_btn_cb_data, volume_up_btn_pressed,
                       BIT(volume_up_btn.pin));
    gpio_add_callback(volume_up_btn.port, &volume_up_btn_cb_data);
	printk("Set up volume up button at %s pin %d\n",
           volume_up_btn.port->name, volume_up_btn.pin);
}

void main(void)
{
    uint32_t i = 0;

    // BOARD_RdcInit();
    BOARD_PeripheralRdcSetting();

    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();

    CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1); /* Set SAI source to AUDIO PLL1 393216000HZ*/
    CLOCK_SetRootDivider(kCLOCK_RootSai3, 1U, 32U);                /* Set root clock to 393216000HZ / 32 = 12.288MHz */
    CLOCK_SetRootMux(kCLOCK_RootI2c3, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootI2c3, 1U, 10U);                  /* Set root clock to 160MHZ / 10 = 16MHZ */
    CLOCK_SetRootMux(kCLOCK_RootGpt1, kCLOCK_GptRootmuxOsc24M);      /* Set GPT source to Osc24 MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootGpt1, 1U, 1U);
    /* Enable the Audio clock on M7 side to make sure the AUDIOMIX domain clock on
     * after A core enters suspend */
    CLOCK_EnableClock(kCLOCK_Audio);
    /* SAI bit clock source */
    AUDIOMIX_AttachClk(AUDIOMIX, kAUDIOMIX_Attach_SAI3_MCLK1_To_SAI3_ROOT);

    /*
     * In order to wakeup M7 from LPM, all PLLCTRLs need to be set to "NeededRun"
     */
    for (i = 0; i != 39; i++)
    {
        CCM->PLL_CTRL[i].PLL_CTRL = kCLOCK_ClockNeededRun;
    }

    printk("\r\n####################  LOW POWER AUDIO TASK ####################\n\r\n");
    printk("    Build Time: %s--%s \r\n", __DATE__, __TIME__);

    // vPortSetupTimerInterrupt();

    APP_SRTM_Init();

    k_thread_create(&app_thread, app_stack, CONFIG_RPMSG_LITE_APP_STACKSIZE,
            app_task, NULL, NULL, NULL,
            -1, K_USER, K_MSEC(0));
    // k_thread_create(&read_buttons_thread, read_buttons_stack, 1024,
    //         read_buttons_task, NULL, NULL, NULL,
    //         -1, K_USER, K_NO_WAIT);
}
