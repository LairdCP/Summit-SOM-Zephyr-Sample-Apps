/*
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "low_power_wakeup.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Globals */
static const struct gpio_dt_spec volume_up_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_UP_BTN_NODE,
                                  gpios,
                                  {0});
static const struct gpio_dt_spec volume_down_btn = GPIO_DT_SPEC_GET_OR(
                                  VOLUME_DOWN_BTN_NODE,
                                  gpios,
                                  {0});
static const struct device *ipm_handle;

static bool vol_down_btn_pushed         = false;
static bool vol_down_btn_pushed_prev    = false;
static bool vol_up_btn_pushed           = false;
static bool vol_up_btn_pushed_prev      = false;

volatile lpw_shell_state_t shell_state  = LPW_SHELL_STATE_UNINITIALIZED;

/* Code */

/**
 * @brief Perform peripheral access configuration via the RDC.
 */
void rdc_configure_peripheral_access(void)
{
    rdc_domain_assignment_t assignment = {0};
    rdc_periph_access_config_t periphConfig;

    assignment.domainId = DOMAIN_ID;

    /* Only configure the RDC if the RDC peripheral write access is allowed. */
    if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC,
                                          assignment.domainId)) != 0U)
    {
        RDC_GetDefaultPeriphAccessConfig(&periphConfig);
        /*
         * Do not allow the A53 domain (domain0) to access the following
         * peripherals.
         */
        periphConfig.policy = RDC_DISABLE_A53_ACCESS;
        periphConfig.periph = kRDC_Periph_UART4;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
        periphConfig.periph = kRDC_Periph_GPT1;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);

        /*
         * Allow read only access to the A53 domain (domain0) for the GPIO3
         * periperal.
         */
        periphConfig.periph = kRDC_Periph_GPIO3;
        periphConfig.policy = RDC_A53_READ_ONLY_ACCESS;
        RDC_SetPeriphAccessConfig(RDC, &periphConfig);
    }
}

/**
 * @brief Perform pre-sleep processing tasks.
 */
void pre_sleep_processing(void)
{
    UART_Deinit((UART_Type *)DT_REG_ADDR(DEBUG_UART_DEVICE));
}

/**
 * @brief Perform post-sleep processing tasks.
 */
void post_sleep_processing(void)
{
	uart_config_t uart_config;
	uint32_t clock_freq;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;

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

/**
 * @brief IPM (MU) interrupt handler.
 * 
 * @param dev       Pointer to device struct
 * @param context   Pointer to context
 * @param id        ID
 * @param data      Pointer to data
 */
void MU1_M7_IRQHandler(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
    uint32_t *data32 = (uint32_t *)data;

    LOG_DBG("Received MU msg: %d", *data32);
}

/**
 * @brief Send a signal via the MU peripheral to wake up the A core.
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
 * @brief Initialize the shell from within a k_work worker.
 * 
 * @param work  Pointer to k_work struct
 */
void shell_init_from_work(struct k_work *work)
{
    if (shell_state == LPW_SHELL_STATE_UNINITIALIZED) {
        shell_state = LPW_SHELL_STATE_INITIALIZING;

        post_sleep_processing();

        const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
        bool log_backend = CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > 0;
        uint32_t level =
            (CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > LOG_LEVEL_DBG) ?
            CONFIG_LOG_MAX_LEVEL : CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL;

        shell_init(shell_backend_uart_get_ptr(), dev,
            shell_backend_uart_get_ptr()->ctx->cfg.flags,
            log_backend, level);

        shell_state = LPW_SHELL_STATE_INITIALIZED;
    }
}

/**
 * @brief Trigger a reinitialization of the shell via a k_work worker.
 */
static void shell_reinit_trigger(void)
{
	static struct k_work shell_init_work;

	k_work_init(&shell_init_work, shell_init_from_work);
	int err = k_work_submit(&shell_init_work);

	(void)err;
	__ASSERT_NO_MSG(err >= 0);
}

/**
 * @brief Callback handler after the shell has been uninitialized.
 * 
 * @param shell Pointer to shell struct
 * @param res   Result from the shell uninitialization
 */
static void shell_uninit_cb(const struct shell *shell, int res)
{
	ARG_UNUSED(shell);

	__ASSERT_NO_MSG(res >= 0);

    pre_sleep_processing();

    /* Flag that the shell is no longer initialized */
    shell_state = LPW_SHELL_STATE_UNINITIALIZED;
}

/**
 * @brief Shell 'demo' command that prints the current board configuration.
 * 
 * @param shell Pointer to shell struct
 * @param argc  Number of arguments received
 * @param argv  Pointer to string array of arguments received
 * 
 * @return      0 for success, otherwise a numerical error code
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
 * @param shell Pointer to shell struct
 * @param argc  Number of arguments received
 * @param argv  Pointer to string array of arguments received
 * 
 * @return      0 for success, otherwise a numerical error code
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
 * @param shell Pointer to shell struct
 * @param argc  Number of arguments received
 * @param argv  Pointer to string array of arguments received
 * @param data  Pointer to received data
 * 
 * @return      0 for success, otherwise a numerical error code
 */
static int cmd_wakeup_a_core(const struct shell *shell, size_t argc,
            char **argv, void *data)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ARG_UNUSED(data);

    wakeup_a_core();

    return 0;
}

/**
 * @brief Shell 'sleep' command that uninitializes the shell UART peripheral and
 * enters the M7 core into a low power state.
 * 
 * @param shell Pointer to shell struct
 * @param argc  Number of arguments received
 * @param argv  Pointer to string array of arguments received
 * @param data  Pointer to received data
 * 
 * @return      0 for success, otherwise a numerical error code
 */
static int cmd_sleep(const struct shell *shell, size_t argc,
            char **argv, void *data)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ARG_UNUSED(data);

	if (shell != shell_backend_uart_get_ptr()) {
		shell_error(shell, "Command dedicated for shell over uart");
		return -EINVAL;
	}

	shell_print(shell,
        "Uninitializing shell, use volume buttons to reinitialize");
	shell_uninit(shell, shell_uninit_cb);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo,
	SHELL_CMD(board, NULL, "Show board name command.", cmd_demo_board),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);

SHELL_CMD_ARG_REGISTER(wakeup, NULL,
    "Wakeup the A core", cmd_wakeup_a_core, 1, 0);

SHELL_CMD_ARG_REGISTER(sleep, NULL,
    "Put M core into low power sleep mode - exit with either volume button",
    cmd_sleep, 1, 0);

/**
 * @brief Volume up button polling timer timeout function which handles checking
 * the volume up button state and triggering the shell re-initialization, if the
 * button is pushed.
 * 
 * @param timer Pointer to timer struct
 */
static void vol_up_btn_poll_timeout(struct k_timer *timer)
{
	ARG_UNUSED(timer);

    /* Check if volume up button was pushed */
    vol_up_btn_pushed = gpio_pin_get_dt(&volume_up_btn) == 1;
    if (vol_up_btn_pushed != vol_up_btn_pushed_prev && vol_up_btn_pushed) {
        LOG_DBG("Detected volume up button push");
        if (shell_state == LPW_SHELL_STATE_UNINITIALIZED) {
            shell_reinit_trigger();
        }
    }
    vol_up_btn_pushed_prev = vol_up_btn_pushed;
}

/**
 * @brief Volume down button polling timer timeout function which handles
 * checking the volume down button state and triggering the shell
 * re-initialization, if the button is pushed.
 * 
 * @param timer Pointer to timer struct
 */
static void vol_down_btn_poll_timeout(struct k_timer *timer)
{
	ARG_UNUSED(timer);

    /* Check if volume down button was pushed */
    vol_down_btn_pushed = gpio_pin_get_dt(&volume_down_btn) == 1;
    if (vol_down_btn_pushed != vol_down_btn_pushed_prev &&
        vol_down_btn_pushed) {
        LOG_DBG("Detected volume down button push");
        if (shell_state == LPW_SHELL_STATE_UNINITIALIZED) {
            shell_reinit_trigger();
        }
    }
    vol_down_btn_pushed_prev = vol_down_btn_pushed;
}

K_TIMER_DEFINE(vol_up_btn_poll_timer, vol_up_btn_poll_timeout, NULL);
K_TIMER_DEFINE(vol_down_btn_poll_timer, vol_down_btn_poll_timeout, NULL);

/**
 * @brief Main function entry point
 * 
 */
void main(void)
{
    uint32_t i = 0;
    int ret;

    /* Use RDC to configure peripheral access */
    rdc_configure_peripheral_access();

    /*
     * Set the ServiceFlag to ServiceBusy so that the A core does not attempt to
     * stop the M7 core when it enters into the DSM state.
     */
    ServiceFlagAddr = ServiceBusy;

    LOG_INF("LOW POWER WAKEUP EXAMPLE");
    LOG_INF("Build Time: %s--%s", __DATE__, __TIME__);

	/* Setup the IPM (MU) peripheral */
	ipm_handle = device_get_binding(CONFIG_IPC_DEV_NAME);
	if (!ipm_handle) {
		LOG_ERR("Failed to find ipm device");
		return;
	}
	if (!device_is_ready(ipm_handle)) {
		while (1) {
            LOG_ERR("MU is not ready");
		}
	}

    ipm_register_callback(ipm_handle, MU1_M7_IRQHandler, NULL);

    ipm_set_enabled(ipm_handle, 1);
    LOG_DBG("IPM initialized");

    /* Setup volume up button */
    if (!device_is_ready(volume_up_btn.port)) {
		LOG_ERR("Error: button device %s is not ready",
		       volume_up_btn.port->name);
		return;
    }
    ret = gpio_pin_configure_dt(&volume_up_btn, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, volume_up_btn.port->name, volume_up_btn.pin);
		return;
	}
	LOG_DBG("Setup volume up button");

    /* Setup volume down button */
    if (!device_is_ready(volume_down_btn.port)) {
		LOG_ERR("Error: button device %s is not ready",
		       volume_down_btn.port->name);
		return;
    }
    ret = gpio_pin_configure_dt(&volume_down_btn, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, volume_down_btn.port->name, volume_down_btn.pin);
		return;
	}
	LOG_DBG("Setup volume down button");

    /* Mark the shell as initialized */
    shell_state = LPW_SHELL_STATE_INITIALIZED;

    /*
     * Start button poll timers. Polling is necessary here because GPIO
     * interrupts are disabled when the A core is put into the DSM state.
     */
    k_timer_start(&vol_up_btn_poll_timer, K_MSEC(100), K_MSEC(100));
    k_timer_start(&vol_down_btn_poll_timer, K_MSEC(100), K_MSEC(100));
}
