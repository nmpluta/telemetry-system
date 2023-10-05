/*
 * Copyright (c) 2022 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief Main function
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include "can_threads.h"
#include "sd_threads.h"
#include "http_threads.h"

#define SLEEP_TIME K_MSEC(250)

extern const struct device * const can_dev;

int main(void)
{
    sd_disk_init();
    sd_threads_init();

    http_threads_init();

    can_init(can_dev);
    can_threads_init();

    while (1)
    {
        k_sleep(SLEEP_TIME);
    }

    return 0;
}
