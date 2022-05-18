#
# Copyright (c) 2022 Laird Connectivity
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

# Prepend the additional boards directory but still allow the original Zephyr
# boards directory to be used with lower priority
list(PREPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

set(rpmsg_lite_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/rpmsg_lite)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("RPMsg-Lite String Echo Example")

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
)

include_directories(
	${rpmsg_lite_dir}/lib/include
	${rpmsg_lite_dir}/lib/include/environment/zephyr
	${rpmsg_lite_dir}/lib/include/platform/imx8mp_m7
	${CMAKE_SOURCE_DIR}/include
)
