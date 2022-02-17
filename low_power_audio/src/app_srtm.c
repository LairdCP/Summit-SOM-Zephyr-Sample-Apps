/*
 * Copyright 2020 NXP
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>

#include "fsl_i2c.h"
#include "fsl_iomuxc.h"
#include "fsl_mu.h"

#include "srtm_audio_service.h"
#include "srtm_dispatcher.h"
#include "srtm_i2c_service.h"
#include "srtm_message.h"
#include "srtm_peercore.h"
#include "srtm_rpmsg_endpoint.h"
#include "srtm_sai_sdma_adapter.h"

#include "app_srtm.h"
#include "low_power_audio.h"
#include "lpm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_MS2TICK(ms) ((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

#define BUFFER_LEN (128 * 1024)
#if (defined(__ICCARM__))
uint8_t g_buffer[BUFFER_LEN] @"AudioBuf";
#else
uint8_t g_buffer[BUFFER_LEN] __attribute__((section("AudioBuf,\"w\",%nobits @")));
#endif
srtm_sai_sdma_local_buf_t g_local_buf = {
    .buf       = (uint8_t *)&g_buffer,
    .bufSize   = BUFFER_LEN,
    .periods   = SRTM_SAI_SDMA_MAX_LOCAL_BUF_PERIODS,
    .threshold = 1,

};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t baseAddr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint8_t len,
                                       uint16_t flags);

static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t baseAddr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint8_t len,
                                        uint16_t flags);

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t baseAddr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel);
/*******************************************************************************
 * Variables
 ******************************************************************************/
app_rpmsg_monitor_t rpmsgMonitor;
volatile app_srtm_state_t srtmState = APP_SRTM_StateRun;

static srtm_dispatcher_t disp;
static srtm_peercore_t core;
static srtm_service_t audioService;
static srtm_service_t i2cService;

struct k_sem monSig;
struct rpmsg_lite_instance *rpmsgHandle;

void *rpmsgMonitorParam;
struct k_timer linkupTimer;

srtm_sai_adapter_t saiAdapter;
static struct _i2c_bus i2c_buses[] = {
    {
        .bus_id         = 2,
        .base_addr      = I2C3_BASE,
        .type           = SRTM_I2C_TYPE_I2C,
        .switch_idx     = APP_I2C_SWITCH_NONE,
        .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED,
    },
};
static struct _i2c_switch i2c_switchs[] = {
    {.slaveAddr = APP_I2C_SWITCH_ADDR, .cur_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED}};

static struct _srtm_i2c_adapter i2c_adapter = {.read          = APP_SRTM_I2C_Read,
                                               .write         = APP_SRTM_I2C_Write,
                                               .switchchannel = APP_SRTM_I2C_SwitchChannel,
                                               .bus_structure = {
                                                   .buses      = i2c_buses,
                                                   .bus_num    = sizeof(i2c_buses) / sizeof(struct _i2c_bus),
                                                   .switches   = i2c_switchs,
                                                   .switch_num = sizeof(i2c_switchs) / sizeof(struct _i2c_switch),
                                               }};

static struct k_thread SRTM_MonitorTask_thread;
static struct k_thread SRTM_DispatcherTask_thread;
K_THREAD_STACK_DEFINE(SRTM_MonitorTask_stack, 2048);
K_THREAD_STACK_DEFINE(SRTM_DispatcherTask_stack, 2048);

/*******************************************************************************
 * Code
 ******************************************************************************/
bool APP_SRTM_ServiceIdle(void)
{
    srtm_audio_state_t TxState, RxState;

    SRTM_SaiSdmaAdapter_GetAudioServiceState(saiAdapter, &TxState, &RxState);
    if (TxState == SRTM_AudioStateClosed && RxState == SRTM_AudioStateClosed)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static uint32_t APP_SRTM_SaiClockSet(mclk_type_t type)
{
    uint32_t sai_source_clk = 0;

    if (type == SRTM_CLK22M)
    {
        CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll2); /* Set SAI source to Audio PLL2 361267200HZ
                                                                          to get 11.2896MHz */
        sai_source_clk = APP_AUDIO_PLL2_FREQ / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) /
                         (CLOCK_GetRootPostDivider(kCLOCK_RootSai3));
    }
    else
    {
        CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1); /* Set SAI source to Audio PLL1 393216000HZ
                                                                          to get 12.288Mhz*/
        sai_source_clk = APP_AUDIO_PLL1_FREQ / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) /
                         (CLOCK_GetRootPostDivider(kCLOCK_RootSai3));
    }
    return sai_source_clk;
}

static void APP_SRTM_ClockGateControl(bool isEnable)
{
    if (isEnable)
    {
        /* Judge if the Audio PLL1/PLL2  are ON, if not, enable them. */
        if ((CCM_ANALOG->AUDIO_PLL1_GEN_CTRL & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK) == 0U)
        {
            *&(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL) =
                CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_RST_MASK | CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
            while (!(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_LOCK_MASK))
            {
            }
        }

        if ((CCM_ANALOG->AUDIO_PLL2_GEN_CTRL & CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK) == 0U)
        {
            *&(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL) =
                CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_RST_MASK | CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK;
            while (!(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL & CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_LOCK_MASK))
            {
            }
        }

        /* Enable the SAI and SDMA clock gate in the AUDIOMIX */
        AUDIOMIX->CLKEN0 |= APP_SAI_GATE | APP_SDMA_GATE;
    }
    else
    {
        /* Judge if the Audio PLL1/PLL2  are OFF, if not, disable them. */
        if (CCM_ANALOG->AUDIO_PLL1_GEN_CTRL & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK)
        {
            *&(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL) &= ~CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
        }

        if (CCM_ANALOG->AUDIO_PLL2_GEN_CTRL & CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK)
        {
            *&(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL) &= ~CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK;
        }

        /* Disable the SAI and SDMA clock gate in the AUDIOMIX */
        AUDIOMIX->CLKEN0 &= ~(APP_SAI_GATE | APP_SDMA_GATE);
    }
}

static void APP_SRTM_PollLinkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (srtmState == APP_SRTM_StateRun)
    {
        if (rpmsg_lite_is_link_up(rpmsgHandle))
        {
            srtmState = APP_SRTM_StateLinkedUp;
            k_sem_give(&monSig);
        }
        else
        {
            /* Start timer to poll linkup status. */
            k_timer_start(&linkupTimer, K_MSEC(APP_LINKUP_TIMER_PERIOD_MS), K_FOREVER);
        }
    }
}

static void APP_LinkupTimerCallback(struct k_timer *xTimer)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_PollLinkup, NULL, NULL);

    if (proc)
    {
        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_SRTM_NotifyPeerCoreReady(struct rpmsg_lite_instance *rpmsgHandle, bool ready)
{
    if (rpmsgMonitor)
    {
        rpmsgMonitor(rpmsgHandle, ready, rpmsgMonitorParam);
    }
}

static void APP_SRTM_Linkup(void)
{
    srtm_channel_t chan;
    srtm_rpmsg_endpoint_config_t rpmsgConfig;

    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, true);

    /* Create SRTM peer core */
    core = SRTM_PeerCore_Create(1U); /* Assign CA53 core ID to 1U */

    SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

    /* Common RPMsg channel config */
    rpmsgConfig.localAddr   = RL_ADDR_ANY;
    rpmsgConfig.peerAddr    = RL_ADDR_ANY;
    rpmsgConfig.rpmsgHandle = rpmsgHandle;

    rpmsgConfig.epName = APP_SRTM_AUDIO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM I2C channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_I2C_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void APP_SRTM_InitPeerCore(void)
{
    copyResourceTable();

    rpmsgHandle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE, RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
    assert(rpmsgHandle);

    if (rpmsg_lite_is_link_up(rpmsgHandle))
    {
        /* If resume context has already linked up, don't need to announce channel
         * again. */
        APP_SRTM_Linkup();
    }
    else
    {
        /* Start timer to poll linkup status. */
        k_timer_start(&linkupTimer, K_MSEC(APP_LINKUP_TIMER_PERIOD_MS), K_FOREVER);
        while (!rpmsg_lite_is_link_up(rpmsgHandle)) {
            ;
        }
    }
}

static void APP_SRTM_InitAudioDevice(bool enable)
{
    if (enable)
    {
        sdma_config_t dmaConfig;

        SDMA_GetDefaultConfig(&dmaConfig);
        dmaConfig.ratio = kSDMA_ARMClockFreq; /* SDMA2 & SDMA3 must set the clock ratio to 1:1. */
        SDMA_Init(APP_SRTM_DMA, &dmaConfig);
    }
    else
    {
        SDMA_Deinit(APP_SRTM_DMA);
    }
}

static void APP_SRTM_InitAudioService(void)
{
    static srtm_sai_sdma_config_t saiTxConfig;

    memset(&saiTxConfig, 0, sizeof(srtm_sai_sdma_config_t));
    /* Create SAI SDMA adapter */
    SAI_GetClassicI2SConfig(&saiTxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo,
                            kSAI_Channel0Mask);              /* SAI channel 0 used by default */
    saiTxConfig.config.syncMode            = kSAI_ModeAsync; /* Tx in async mode */
    saiTxConfig.dataLine1                  = 0U;
    saiTxConfig.config.fifo.fifoWatermark  = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
    saiTxConfig.mclkConfig.mclkSourceClkHz = APP_SAI_CLK_FREQ;
    saiTxConfig.mclkConfig.mclkHz          = saiTxConfig.mclkConfig.mclkSourceClkHz; /* Set the output mclk equal to
                                                                                        its source clk by default */
    saiTxConfig.mclkConfig.mclkOutputEnable = true;                                  /* Enable the MCLK output */

    saiTxConfig.stopOnSuspend = true; /* Audio data is in DRAM which is not accessable in A53 suspend. */
    saiTxConfig.threshold     = 1U;   /* Under the threshold value would trigger
                                         periodDone message to A53 */
    saiTxConfig.guardTime = 2000;     /* Unit:ms. This is a lower limit that M core should
                                         reserve such time data to wakeup A core. */
    saiTxConfig.dmaChannel                 = APP_SAI_TX_DMA_CHANNEL;
    saiTxConfig.ChannelPriority            = APP_SAI_TX_DMA_CHANNEL_PRIORITY;
    saiTxConfig.eventSource                = APP_SAI_TX_DMA_SOURCE;
    saiTxConfig.extendConfig.clkSetting    = APP_SRTM_SaiClockSet;
    saiTxConfig.extendConfig.clkGate       = APP_SRTM_ClockGateControl;
    saiTxConfig.extendConfig.audioDevInit  = APP_SRTM_InitAudioDevice;
    saiTxConfig.extendConfig.dsdSaiSetting = NULL;
    saiTxConfig.extendConfig.pcmSaiSetting = NULL;

    saiAdapter = SRTM_SaiSdmaAdapter_Create(APP_SRTM_SAI, APP_SRTM_DMA, &saiTxConfig, NULL);
    assert(saiAdapter);

    /*  Set SAI DMA IRQ Priority. */
    NVIC_SetPriority(APP_SRTM_DMA_IRQn, APP_SAI_TX_DMA_IRQ_PRIO);
    NVIC_SetPriority(APP_SRTM_SAI_IRQn, APP_SAI_IRQ_PRIO);

    /* Create and register audio service */
    SRTM_SaiSdmaAdapter_SetTxLocalBuf(saiAdapter, &g_local_buf);
    audioService = SRTM_AudioService_Create(saiAdapter, NULL);
    SRTM_Dispatcher_RegisterService(disp, audioService);
}

static void APP_SRTM_InitI2CDevice(void)
{
    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = APP_I2C_BAUDRATE;
    I2C_MasterInit(CODEC_I2C, &masterConfig, CODEC_I2C_CLOCK_FREQ);
}

static void APP_SRTM_InitI2CService(void)
{
    APP_SRTM_InitI2CDevice();
    i2cService = SRTM_I2CService_Create(&i2c_adapter);
    SRTM_Dispatcher_RegisterService(disp, i2cService);
}

static void APP_SRTM_InitServices(void)
{
    APP_SRTM_InitI2CService();
    APP_SRTM_InitAudioService();
}

static void SRTM_MonitorTask(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Initialize services and add to dispatcher */
    APP_SRTM_InitServices();

    /* Start SRTM dispatcher */
    SRTM_Dispatcher_Start(disp);

    k_sem_give(&monSig);
    while (true)
    {
        k_sem_take(&monSig, K_FOREVER);
        if (srtmState == APP_SRTM_StateRun)
        {
            SRTM_Dispatcher_Stop(disp);
            APP_SRTM_InitPeerCore();
            SRTM_Dispatcher_Start(disp);
        }
        else
        {
            SRTM_Dispatcher_Stop(disp);
            /* Need to announce channel as we just linked up. */
            APP_SRTM_Linkup();
            SRTM_Dispatcher_Start(disp);
        }
    }
}

static void SRTM_DispatcherTask(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    SRTM_Dispatcher_Run(disp);
}

void APP_SRTM_Init(void)
{
    int ret;
    
    ret = k_sem_init(&monSig, 0, 1);
    assert(ret == 0);
    k_timer_init(&linkupTimer, APP_LinkupTimerCallback, NULL);
    /* Create SRTM dispatcher */
    disp = SRTM_Dispatcher_Create();

    k_thread_create(&SRTM_MonitorTask_thread, SRTM_MonitorTask_stack, 2048,
            SRTM_MonitorTask, NULL, NULL, NULL,
            APP_SRTM_MONITOR_TASK_PRIO, K_USER, K_NO_WAIT);

    k_thread_create(&SRTM_DispatcherTask_thread, SRTM_DispatcherTask_stack, 2048,
            SRTM_DispatcherTask, NULL, NULL, NULL,
            APP_SRTM_DISPATCHER_TASK_PRIO, K_USER, K_NO_WAIT);
}

void APP_SRTM_Suspend(void)
{
    /* For user use. */
}

void APP_SRTM_Resume(void)
{
    /* For user use. */
}

static srtm_status_t APP_SRTM_I2C_SwitchChannel(srtm_i2c_adapter_t adapter,
                                                uint32_t base_addr,
                                                srtm_i2c_type_t type,
                                                uint16_t slaveAddr,
                                                srtm_i2c_switch_channel channel)
{
    uint8_t txBuff[1];
    assert(channel < SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED);
    txBuff[0] = 1 << (uint8_t)channel;
    return adapter->write(adapter, base_addr, type, slaveAddr, txBuff, sizeof(txBuff),
                          SRTM_I2C_FLAG_NEED_STOP); // APP_SRTM_I2C_Write
}

static status_t SRTM_I2C_Send(I2C_Type *base, uint8_t slaveAddr, uint8_t *buf, uint8_t len, uint16_t flags)
{
    status_t reVal = kStatus_Success;
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0U, sizeof(i2c_master_transfer_t));

    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction    = kI2C_Write;
    masterXfer.data         = buf;
    masterXfer.dataSize     = len;
    masterXfer.flags        = kI2C_TransferDefaultFlag;

    LPM_IncreseBlockSleepCnt();
    reVal = I2C_MasterTransferBlocking(base, &masterXfer);
    LPM_DecreaseBlockSleepCnt();
    return reVal;
}

static status_t SRTM_I2C_Read(I2C_Type *base, uint8_t slaveAddr, uint8_t *buf, uint8_t len, uint16_t flags)
{
    status_t reVal = kStatus_Success;

    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0U, sizeof(i2c_master_transfer_t));

    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction    = kI2C_Read;
    masterXfer.data         = buf;
    masterXfer.dataSize     = len;
    masterXfer.flags        = kI2C_TransferDefaultFlag;

    LPM_IncreseBlockSleepCnt();
    reVal = I2C_MasterTransferBlocking(base, &masterXfer);
    LPM_DecreaseBlockSleepCnt();

    return reVal;
}
static srtm_status_t APP_SRTM_I2C_Write(srtm_i2c_adapter_t adapter,
                                        uint32_t baseAddr,
                                        srtm_i2c_type_t type,
                                        uint16_t slaveAddr,
                                        uint8_t *buf,
                                        uint8_t len,
                                        uint16_t flags)
{
    status_t retVal = kStatus_Success;

    switch (type)
    {
        case SRTM_I2C_TYPE_I2C:
            retVal = SRTM_I2C_Send((I2C_Type *)baseAddr, slaveAddr, buf, len, flags);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t APP_SRTM_I2C_Read(srtm_i2c_adapter_t adapter,
                                       uint32_t baseAddr,
                                       srtm_i2c_type_t type,
                                       uint16_t slaveAddr,
                                       uint8_t *buf,
                                       uint8_t len,
                                       uint16_t flags)
{
    status_t retVal = kStatus_Success;
    switch (type)
    {
        case SRTM_I2C_TYPE_I2C:
            retVal = SRTM_I2C_Read((I2C_Type *)baseAddr, slaveAddr, buf, len, flags);
            break;
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}