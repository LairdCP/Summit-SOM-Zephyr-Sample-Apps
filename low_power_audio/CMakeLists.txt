#
# Copyright (c) 2022 Laird Connectivity
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

# Prepend the additional boards directory but still allow the original Zephyr
# boards directory to be used with lower priority
list(PREPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

set(modules_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../modules)
set(rpmsg_lite_dir ${modules_dir}/rpmsg_lite)
set(MCUX_SDK_PROJECT_NAME app)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Low Power Audio Example")

target_sources(app PRIVATE
    ${rpmsg_lite_dir}/lib/common/llist.c
    ${rpmsg_lite_dir}/lib/rpmsg_lite/rpmsg_lite.c
    ${rpmsg_lite_dir}/lib/rpmsg_lite/rpmsg_ns.c
    ${rpmsg_lite_dir}/lib/rpmsg_lite/rpmsg_queue.c
    ${rpmsg_lite_dir}/lib/virtio/virtqueue.c
    ${CMAKE_SOURCE_DIR}/src/main.c
    ${CMAKE_SOURCE_DIR}/src/rsc_table.c
    ${CMAKE_SOURCE_DIR}/src/rpmsg_env_zephyr.c
    ${CMAKE_SOURCE_DIR}/src/rpmsg_platform.c
    ${CMAKE_SOURCE_DIR}/src/lpm.c
    ${CMAKE_SOURCE_DIR}/src/app_srtm.c
    ${CMAKE_SOURCE_DIR}/src/srtm_heap_zephyr.c
    ${CMAKE_SOURCE_DIR}/src/srtm_mutex_zephyr.c
    ${CMAKE_SOURCE_DIR}/src/srtm_sem_zephyr.c

    # Pull in srtm_message_pool.c from the MCUX SDK since it isn't dependent on
    # FreeRTOS
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/port/srtm_message_pool.c

    # Workaround until the GPC driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/gpc_2/fsl_gpc.c

    # Workaround until the GPT driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/gpt/fsl_gpt.c

    # Workaround until the SAI SDMA driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/sai/fsl_sai_sdma.c
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/sai/fsl_sai.c
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/sdma/fsl_sdma.c

    # Pull in SRTM services/channels
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/services/srtm_audio_service.c
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/services/srtm_sai_sdma_adapter.c
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/services/srtm_i2c_service.c
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/channels/srtm_rpmsg_endpoint.c
)

include_directories(
	${rpmsg_lite_dir}/lib/include
	${rpmsg_lite_dir}/lib/include/environment/zephyr
    ${rpmsg_lite_dir}/lib/include/platform/imx8mp_m7
    ${CMAKE_SOURCE_DIR}/include

    # Workaround until the GPC driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/gpc_2

    # Workaround until the GPT driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/gpt

    # Workaround until the SAI SDMA driver is properly pulled in to the NXP HAL
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/sai
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/sdma

    # Pull in SRTM services/channels
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/services
    ${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/channels
)

# Include other MCUX drivers/components
include(${modules_dir}/hal/nxp/mcux/mcux-sdk/components/srtm/driver_srtm_MIMX8ML8.cmake)
include(${modules_dir}/hal/nxp/mcux/mcux-sdk/drivers/ii2c/driver_ii2c.cmake)