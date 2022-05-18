/*
 * Copyright 2020 NXP
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>

#include <drivers/i2c.h>
#include <devicetree.h>
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
#define I2C_DEVICE_NAME DT_PROP(DT_NODELABEL(i2c3), label)

#define BUFFER_LEN (128 * 1024)
#if (defined(__ICCARM__))
uint8_t g_buffer[BUFFER_LEN] @"AudioBuf";
#else
uint8_t g_buffer[BUFFER_LEN] __attribute__((section("AudioBuf,\"w\",%nobits @")));
#endif
srtm_sai_sdma_local_buf_t g_local_buf = {
	.buf = (uint8_t *)&g_buffer,
	.bufSize = BUFFER_LEN,
	.periods = SRTM_SAI_SDMA_MAX_LOCAL_BUF_PERIODS,
	.threshold = 1,

};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static srtm_status_t app_srtm_i2c_read(srtm_i2c_adapter_t adapter, uint32_t base_addr,
				       srtm_i2c_type_t type, uint16_t slave_addr, uint8_t *buf,
				       uint8_t len, uint16_t flags);

static srtm_status_t app_srtm_i2c_write(srtm_i2c_adapter_t adapter, uint32_t base_addr,
					srtm_i2c_type_t type, uint16_t slave_addr, uint8_t *buf,
					uint8_t len, uint16_t flags);

static srtm_status_t app_srtm_i2c_switch_channel(srtm_i2c_adapter_t adapter, uint32_t base_addr,
						 srtm_i2c_type_t type, uint16_t slave_addr,
						 srtm_i2c_switch_channel channel);

extern void SDMA3_DriverIRQHandler(void);
extern void I2S3_DriverIRQHandler(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
app_rpmsg_monitor_t rpmsg_monitor;
volatile app_srtm_state_t srtm_state = APP_SRTM_STATE_RUN;

static srtm_dispatcher_t disp;
static srtm_peercore_t core;
static srtm_service_t audio_service;
static srtm_service_t i2c_service;

struct k_sem mon_sig;
struct rpmsg_lite_instance *rpmsg_handle;

void *rpmsg_monitor_param;
struct k_timer linkup_timer;

srtm_sai_adapter_t sai_adapter;
static struct _i2c_bus i2c_buses[] = {
	{
		.bus_id = 2,
		.base_addr = I2C3_BASE,
		.type = SRTM_I2C_TYPE_I2C,
		.switch_idx = APP_I2C_SWITCH_NONE,
		.switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED,
	},
};
static struct _i2c_switch i2c_switchs[] = {
	{ .slaveAddr = APP_I2C_SWITCH_ADDR, .cur_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED }
};

static struct _srtm_i2c_adapter i2c_adapter = { .read = app_srtm_i2c_read,
						.write = app_srtm_i2c_write,
						.switchchannel = app_srtm_i2c_switch_channel,
						.bus_structure = {
							.buses = i2c_buses,
							.bus_num = sizeof(i2c_buses) /
								   sizeof(struct _i2c_bus),
							.switches = i2c_switchs,
							.switch_num = sizeof(i2c_switchs) /
								      sizeof(struct _i2c_switch),
						} };

static struct k_thread srtm_monitor_task_thread;
static struct k_thread srtm_dispatcher_task_thread;
K_THREAD_STACK_DEFINE(srtm_monitor_task_stack, 2048);
K_THREAD_STACK_DEFINE(srtm_dispatcher_task_stack, 2048);

/*******************************************************************************
 * Code
 ******************************************************************************/
bool app_srtm_service_idle(void)
{
	srtm_audio_state_t tx_state, rx_state;

	SRTM_SaiSdmaAdapter_GetAudioServiceState(sai_adapter, &tx_state, &rx_state);
	if (tx_state == SRTM_AudioStateClosed && rx_state == SRTM_AudioStateClosed) {
		return true;
	} else {
		return false;
	}
}

static uint32_t app_srtm_sai_clock_set(mclk_type_t type)
{
	uint32_t sai_source_clk = 0;

	if (type == SRTM_CLK22M) {
		/* Set SAI source to Audio PLL2 361267200HZ to get 11.2896MHz */
		CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll2);
		sai_source_clk = APP_AUDIO_PLL2_FREQ / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) /
				 (CLOCK_GetRootPostDivider(kCLOCK_RootSai3));
	} else {
		/* Set SAI source to Audio PLL1 393216000HZ to get 12.288Mhz */
		CLOCK_SetRootMux(kCLOCK_RootSai3, kCLOCK_SaiRootmuxAudioPll1);
		sai_source_clk = APP_AUDIO_PLL1_FREQ / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) /
				 (CLOCK_GetRootPostDivider(kCLOCK_RootSai3));
	}
	return sai_source_clk;
}

static void app_srtm_clock_gate_control(bool is_enable)
{
	if (is_enable) {
		/* Judge if the Audio PLL1/PLL2  are ON, if not, enable them. */
		if ((CCM_ANALOG->AUDIO_PLL1_GEN_CTRL &
		     CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK) == 0U) {
			*&(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL) =
				CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_RST_MASK |
				CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
			while (!(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL &
				 CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_LOCK_MASK)) {
			}
		}

		if ((CCM_ANALOG->AUDIO_PLL2_GEN_CTRL &
		     CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK) == 0U) {
			*&(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL) =
				CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_RST_MASK |
				CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK;
			while (!(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL &
				 CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_LOCK_MASK)) {
			}
		}

		/* Enable the SAI and SDMA clock gate in the AUDIOMIX */
		AUDIOMIX->CLKEN0 |= APP_SAI_GATE | APP_SDMA_GATE;
	} else {
		/* Judge if the Audio PLL1/PLL2  are OFF, if not, disable them. */
		if (CCM_ANALOG->AUDIO_PLL1_GEN_CTRL &
		    CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK) {
			*&(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL) &=
				~CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
		}

		if (CCM_ANALOG->AUDIO_PLL2_GEN_CTRL &
		    CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK) {
			*&(CCM_ANALOG->AUDIO_PLL2_GEN_CTRL) &=
				~CCM_ANALOG_AUDIO_PLL2_GEN_CTRL_PLL_CLKE_MASK;
		}

		/* Disable the SAI and SDMA clock gate in the AUDIOMIX */
		AUDIOMIX->CLKEN0 &= ~(APP_SAI_GATE | APP_SDMA_GATE);
	}
}

static void app_srtm_poll_linkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
	ARG_UNUSED(dispatcher);
	ARG_UNUSED(param1);
	ARG_UNUSED(param2);

	if (srtm_state == APP_SRTM_STATE_RUN) {
		if (rpmsg_lite_is_link_up(rpmsg_handle)) {
			srtm_state = APP_SRTM_STATE_LINKED_UP;
			k_sem_give(&mon_sig);
		} else {
			/* Start timer to poll linkup status. */
			k_timer_start(&linkup_timer, K_MSEC(APP_LINKUP_TIMER_PERIOD_MS), K_FOREVER);
		}
	}
}

static void app_linkup_timer_callback(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	srtm_procedure_t proc = SRTM_Procedure_Create(app_srtm_poll_linkup, NULL, NULL);

	if (proc) {
		SRTM_Dispatcher_PostProc(disp, proc);
	}
}

static void app_srtm_notify_peer_core_ready(struct rpmsg_lite_instance *rpmsg_handle, bool ready)
{
	if (rpmsg_monitor) {
		rpmsg_monitor(rpmsg_handle, ready, rpmsg_monitor_param);
	}
}

static void app_srtm_linkup(void)
{
	srtm_channel_t chan;
	srtm_rpmsg_endpoint_config_t rpmsg_config;

	app_srtm_notify_peer_core_ready(rpmsg_handle, true);

	/* Create SRTM peer core */
	/* Assign CA53 core ID to 1U */
	core = SRTM_PeerCore_Create(1U);

	SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

	/* Common RPMsg channel config */
	rpmsg_config.localAddr = RL_ADDR_ANY;
	rpmsg_config.peerAddr = RL_ADDR_ANY;
	rpmsg_config.rpmsgHandle = rpmsg_handle;

	rpmsg_config.epName = APP_SRTM_AUDIO_CHANNEL_NAME;
	chan = SRTM_RPMsgEndpoint_Create(&rpmsg_config);
	SRTM_PeerCore_AddChannel(core, chan);

	/* Create and add SRTM I2C channel to peer core*/
	rpmsg_config.epName = APP_SRTM_I2C_CHANNEL_NAME;
	chan = SRTM_RPMsgEndpoint_Create(&rpmsg_config);
	SRTM_PeerCore_AddChannel(core, chan);

	SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void app_srtm_init_peer_core(void)
{
	copy_resource_table();

	rpmsg_handle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE,
					      RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
	assert(rpmsg_handle);

	if (rpmsg_lite_is_link_up(rpmsg_handle)) {
		/* If resume context has already linked up, don't need to announce channel again. */
		app_srtm_linkup();
	} else {
		/* Start timer to poll linkup status. */
		k_timer_start(&linkup_timer, K_MSEC(APP_LINKUP_TIMER_PERIOD_MS), K_FOREVER);
		while (!rpmsg_lite_is_link_up(rpmsg_handle)) {
			;
		}
	}
}

static void app_srtm_init_audio_device(bool enable)
{
	if (enable) {
		sdma_config_t dmaConfig;

		SDMA_GetDefaultConfig(&dmaConfig);
		/* SDMA2 & SDMA3 must set the clock ratio to 1:1. */
		dmaConfig.ratio = kSDMA_ARMClockFreq;
		SDMA_Init(APP_SRTM_DMA, &dmaConfig);
	} else {
		SDMA_Deinit(APP_SRTM_DMA);
	}
}

static void app_srtm_init_audio_service(void)
{
	static srtm_sai_sdma_config_t sai_tx_config;

	memset(&sai_tx_config, 0, sizeof(srtm_sai_sdma_config_t));
	/* Create SAI SDMA adapter */
	/* SAI channel 0 used by default */
	SAI_GetClassicI2SConfig(&sai_tx_config.config, kSAI_WordWidth16bits, kSAI_Stereo,
				kSAI_Channel0Mask);
	/* Tx in async mode */
	sai_tx_config.config.syncMode = kSAI_ModeAsync;
	sai_tx_config.dataLine1 = 0U;
	sai_tx_config.config.fifo.fifoWatermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
	sai_tx_config.mclkConfig.mclkSourceClkHz = APP_SAI_CLK_FREQ;
	/* Set the output mclk equal to its source clk by default */
	sai_tx_config.mclkConfig.mclkHz = sai_tx_config.mclkConfig.mclkSourceClkHz;

	/* Enable the MCLK output */
	sai_tx_config.mclkConfig.mclkOutputEnable = true;

	/* Audio data is in DRAM which is not accessable in A53 suspend. */
	sai_tx_config.stopOnSuspend = true;
	/* Under the threshold value would trigger periodDone message to A53 */
	sai_tx_config.threshold = 1U;
	/*
	 * Unit:ms. This is a lower limit that M core should reserve such time data to wakeup A
	 * core.
	 */
	sai_tx_config.guardTime = 2000;
	sai_tx_config.dmaChannel = APP_SAI_TX_DMA_CHANNEL;
	sai_tx_config.ChannelPriority = APP_SAI_TX_DMA_CHANNEL_PRIORITY;
	sai_tx_config.eventSource = APP_SAI_TX_DMA_SOURCE;
	sai_tx_config.extendConfig.clkSetting = app_srtm_sai_clock_set;
	sai_tx_config.extendConfig.clkGate = app_srtm_clock_gate_control;
	sai_tx_config.extendConfig.audioDevInit = app_srtm_init_audio_device;
	sai_tx_config.extendConfig.dsdSaiSetting = NULL;
	sai_tx_config.extendConfig.pcmSaiSetting = NULL;

	sai_adapter = SRTM_SaiSdmaAdapter_Create(APP_SRTM_SAI, APP_SRTM_DMA, &sai_tx_config, NULL);
	assert(sai_adapter);

	/*  Set SAI DMA IRQ Priority. */
	IRQ_CONNECT(APP_SRTM_DMA_IRQn, APP_SAI_TX_DMA_IRQ_PRIO, SDMA3_DriverIRQHandler, NULL, 0);
	IRQ_CONNECT(APP_SRTM_SAI_IRQn, APP_SAI_IRQ_PRIO, I2S3_DriverIRQHandler, NULL, 0);

	/* Create and register audio service */
	SRTM_SaiSdmaAdapter_SetTxLocalBuf(sai_adapter, &g_local_buf);
	audio_service = SRTM_AudioService_Create(sai_adapter, NULL);
	SRTM_Dispatcher_RegisterService(disp, audio_service);
}

static void app_srtm_init_i2c_device(void)
{
	/* In Zephyr, this is handled by the I2C driver itself */
}

static void app_srtm_init_i2c_service(void)
{
	app_srtm_init_i2c_device();
	i2c_service = SRTM_I2CService_Create(&i2c_adapter);
	SRTM_Dispatcher_RegisterService(disp, i2c_service);
}

static void app_srtm_init_services(void)
{
	app_srtm_init_i2c_service();
	app_srtm_init_audio_service();
}

static void srtm_monitor_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Initialize services and add to dispatcher */
	app_srtm_init_services();

	/* Start SRTM dispatcher */
	SRTM_Dispatcher_Start(disp);

	k_sem_give(&mon_sig);
	while (true) {
		k_sem_take(&mon_sig, K_FOREVER);
		if (srtm_state == APP_SRTM_STATE_RUN) {
			SRTM_Dispatcher_Stop(disp);
			app_srtm_init_peer_core();
			SRTM_Dispatcher_Start(disp);
		} else {
			SRTM_Dispatcher_Stop(disp);
			/* Need to announce channel as we just linked up. */
			app_srtm_linkup();
			SRTM_Dispatcher_Start(disp);
		}
	}
}

static void srtm_dispatcher_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	SRTM_Dispatcher_Run(disp);
}

void app_srtm_init(void)
{
	int ret;

	ret = k_sem_init(&mon_sig, 0, 1);
	assert(ret == 0);
	k_timer_init(&linkup_timer, app_linkup_timer_callback, NULL);
	/* Create SRTM dispatcher */
	disp = SRTM_Dispatcher_Create();

	k_thread_create(&srtm_monitor_task_thread, srtm_monitor_task_stack, 2048, srtm_monitor_task,
			NULL, NULL, NULL, APP_SRTM_MONITOR_TASK_PRIO, K_USER, K_NO_WAIT);

	k_thread_create(&srtm_dispatcher_task_thread, srtm_dispatcher_task_stack, 2048,
			srtm_dispatcher_task, NULL, NULL, NULL, APP_SRTM_DISPATCHER_TASK_PRIO,
			K_USER, K_NO_WAIT);
}

void app_srtm_suspend(void)
{
	/* For user use. */
}

void app_srtm_resume(void)
{
	/* For user use. */
}

static srtm_status_t app_srtm_i2c_switch_channel(srtm_i2c_adapter_t adapter, uint32_t base_addr,
						 srtm_i2c_type_t type, uint16_t slave_addr,
						 srtm_i2c_switch_channel channel)
{
	uint8_t tx_buff[1];
	assert(channel < SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED);
	tx_buff[0] = 1 << (uint8_t)channel;
	return adapter->write(adapter, base_addr, type, slave_addr, tx_buff, sizeof(tx_buff),
			      SRTM_I2C_FLAG_NEED_STOP);
}

static status_t srtm_i2c_send(I2C_Type *base, uint8_t slave_addr, uint8_t *buf, uint8_t len,
			      uint16_t flags)
{
	ARG_UNUSED(base);
	ARG_UNUSED(flags);

	status_t ret_val = kStatus_Success;

	lpm_increase_block_sleep_count();
	ret_val = i2c_write(device_get_binding(I2C_DEVICE_NAME), buf, len, slave_addr);
	lpm_decrease_block_sleep_count();
	return ret_val;
}

static status_t srtm_i2c_read(I2C_Type *base, uint8_t slave_addr, uint8_t *buf, uint8_t len,
			      uint16_t flags)
{
	ARG_UNUSED(base);
	ARG_UNUSED(flags);

	status_t ret_val = kStatus_Success;

	lpm_increase_block_sleep_count();
	ret_val = i2c_read(device_get_binding(I2C_DEVICE_NAME), buf, len, slave_addr);
	lpm_decrease_block_sleep_count();

	return ret_val;
}

static srtm_status_t app_srtm_i2c_write(srtm_i2c_adapter_t adapter, uint32_t base_addr,
					srtm_i2c_type_t type, uint16_t slave_addr, uint8_t *buf,
					uint8_t len, uint16_t flags)
{
	status_t ret_val = kStatus_Success;

	switch (type) {
	case SRTM_I2C_TYPE_I2C:
		ret_val = srtm_i2c_send((I2C_Type *)base_addr, slave_addr, buf, len, flags);
		break;
	default:
		break;
	}
	return (ret_val == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t app_srtm_i2c_read(srtm_i2c_adapter_t adapter, uint32_t base_addr,
				       srtm_i2c_type_t type, uint16_t slave_addr, uint8_t *buf,
				       uint8_t len, uint16_t flags)
{
	status_t ret_val = kStatus_Success;
	switch (type) {
	case SRTM_I2C_TYPE_I2C:
		ret_val = srtm_i2c_read((I2C_Type *)base_addr, slave_addr, buf, len, flags);
		break;
	default:
		break;
	}
	return (ret_val == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}
