#
# Copyright (c) 2022 Laird Connectivity
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

# Prepend the additional boards directory but still allow the original Zephyr
# boards directory to be used with lower priority
list(PREPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

set(MCUX_SDK_PROJECT_NAME app)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Buttons Example")

target_sources(app PRIVATE
    ${CMAKE_SOURCE_DIR}/src/main.c
)
