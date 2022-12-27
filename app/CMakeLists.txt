#-------------------------------------------------------------------------------
# Zephyr OBD2-Logger Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C VERSION 1.0.0)

set(BOARD_FLASH_RUNNER jlink)

configure_file(app_version.h.in ${CMAKE_BINARY_DIR}/app/include/app_version.h)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/app/include src)

target_sources(app PRIVATE src/main.c
                           src/can_threads.c src/can_threads.h
                           src/sd_threads.c src/sd_threads.h
                           src/http_threads.c src/http_threads.h
                           src/sockets.c src/sockets.h)
