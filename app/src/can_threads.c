/*
 * Copyright (c) 2022 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file can_threads.c
 * @brief Can related threads
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#include "can_threads.h"

#define CAN_STATE_THREAD_STACK_SIZE 512
#define CAN_STATE_THREAD_PRIORITY   2
#define LED_MSG_ID                  0x10
#define COUNTER_MSG_ID              0x12345
#define SLEEP_TIME                  K_MSEC(250)

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY   2

#define TX_THREAD_STACK_SIZE 512
#define TX_THREAD_PRIORITY   2

#define CAN_STATE_THREAD_STACK_SIZE 512
#define CAN_STATE_THREAD_PRIORITY   2

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_state_stack, CAN_STATE_THREAD_STACK_SIZE);

struct k_thread rx_thread_data;
struct k_thread tx_thread_data;
struct k_thread can_state_thread_data;
struct k_work   state_change_work;

k_tid_t rx_tid;
k_tid_t tx_tid;
k_tid_t can_state_tid;

const struct device * const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

enum can_state         current_state;
struct can_bus_err_cnt current_err_cnt;

CAN_MSGQ_DEFINE(counter_msgq, 2);

/**
 * It prints an error message
 * if the error code is not zero
 *
 * @param dev The device that triggered the callback.
 * @param error The error code.
 * @param arg A pointer to the argument passed to the callback function.
 */
void tx_irq_callback(const struct device * dev, int error, void * arg)
{
    char * sender = (char *)arg;

    ARG_UNUSED(dev);

    if (0 != error)
    {
        printf("Callback! error-code: %d\nSender: %s\n", error, sender);
    }
}

/**
 * It takes a state and returns a string representation of that state
 *
 * @param state The current state of the CAN controller.
 *
 * @return A string representation of the state.
 */
char * state_to_str(enum can_state state)
{
    switch (state)
    {
        case CAN_STATE_ERROR_ACTIVE:
            return "error-active";
        case CAN_STATE_ERROR_WARNING:
            return "error-warning";
        case CAN_STATE_ERROR_PASSIVE:
            return "error-passive";
        case CAN_STATE_BUS_OFF:
            return "bus-off";
        case CAN_STATE_STOPPED:
            return "stopped";
        default:
            return "unknown";
    }
}

void rx_thread(void * arg1, void * arg2, void * arg3)
{
    printf("Initialization of rx_thread.\n");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    const struct can_filter filter = {.flags = CAN_FILTER_DATA | CAN_FILTER_IDE,
                                      .id    = COUNTER_MSG_ID,
                                      .mask  = CAN_EXT_ID_MASK};

    struct can_frame frame;
    int              filter_id;

    filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
    printf("Counter filter id: %d\n", filter_id);

    while (1)
    {
        k_msgq_get(&counter_msgq, &frame, K_FOREVER);

        if (frame.dlc != 2U)
        {
            printf("Wrong data length: %u\n", frame.dlc);
            continue;
        }

        printf("Counter received: %u\n", sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
        k_sleep(SLEEP_TIME);
    }
}

void tx_thread(void * arg1, void * arg2, void * arg3)
{
    printf("Initialization of tx_thread.\n");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    int ret;
    (void)ret;

    struct can_frame frame = {
        .id   = 0x123,
        .dlc  = 8,
        .data = {1, 2, 3, 4, 5, 6, 7, 8},
    };

    while (1)
    {
        /* A variable that stores the return value of the function. */
        ret = can_send(can_dev, &frame, K_FOREVER, tx_irq_callback, NULL);
        if (ret != 0)
        {
            printf("Sending failed [%d]\n", ret);
        }
        else
        {
            printf("Message id = [%d] successfully sended.\n", frame.id);
        }
        k_sleep(SLEEP_TIME);
    }
}

void can_state_thread(void * unused1, void * unused2, void * unused3)
{
    printf("Initialization of can_state_thread.\n");

    struct can_bus_err_cnt err_cnt      = {0, 0};
    struct can_bus_err_cnt err_cnt_prev = {0, 0};
    enum can_state         state_prev   = CAN_STATE_ERROR_ACTIVE;
    enum can_state         state;
    int                    err;

    while (1)
    {
        err = can_get_state(can_dev, &state, &err_cnt);
        if (0 != err)
        {
            printf("Failed to get CAN controller state: %d", err);
            k_sleep(K_MSEC(100));
            continue;
        }

        if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
            err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt || state_prev != state)
        {

            err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
            err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
            state_prev              = state;
            printf("state: %s\n"
                   "rx error count: %d\n"
                   "tx error count: %d\n",
                   state_to_str(state),
                   err_cnt.rx_err_cnt,
                   err_cnt.tx_err_cnt);
        }
        else
        {
            k_sleep(K_MSEC(100));
        }
    }
}

void state_change_work_handler(struct k_work * work)
{
    printf("State Change ISR\nstate: %s\n"
           "rx error count: %d\n"
           "tx error count: %d\n",
           state_to_str(current_state),
           current_err_cnt.rx_err_cnt,
           current_err_cnt.tx_err_cnt);

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    if (current_state == CAN_STATE_BUS_OFF)
    {
        printf("Recover from bus-off\n");

        if (can_recover(can_dev, K_MSEC(100)) != 0)
        {
            printf("Recovery timed out\n");
        }
    }
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
}

void state_change_callback(const struct device *  dev,
                           enum can_state         state,
                           struct can_bus_err_cnt err_cnt,
                           void *                 user_data)
{
    struct k_work * work = (struct k_work *)user_data;

    ARG_UNUSED(dev);

    current_state   = state;
    current_err_cnt = err_cnt;
    k_work_submit(work);
}

/**
 * It initializes the CAN controller
 *
 * @param can_dev The device structure of the CAN controller.
 *
 * @return The return value is the error code.
 */
void can_init(const struct device * can_dev)
{
    int ret;
    (void)ret;
    struct can_timing timing;

    ret = device_is_ready(can_dev);
    if (false == ret)
    {
        printf("CAN: Device %s not ready.\n", can_dev->name);
    }

    ret = can_calc_timing(can_dev, &timing, 500000, 875);
    if (ret > 0)
    {
        printf("Sample-Point error: %d", ret);
    }

    if (ret < 0)
    {
        printf("Failed to calc a valid timing");
        return;
    }

    ret = can_set_timing(can_dev, &timing);
    if (0 != ret)
    {
        printf("Failed to set timing");
    }

    ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
    if (0 != ret)
    {
        printf("Failed to set CAN controller operation mode");
    }

    ret = can_start(can_dev);
    if (0 != ret)
    {
        printf("Failed to start CAN controller");
    }

    printf("CAN: Device %s initialization finished.\n", can_dev->name);
}

void can_threads_init(void)
{
    k_work_init(&state_change_work, state_change_work_handler);

    can_state_tid = k_thread_create(&can_state_thread_data,
                                    can_state_stack,
                                    K_THREAD_STACK_SIZEOF(can_state_stack),
                                    can_state_thread,
                                    NULL,
                                    NULL,
                                    NULL,
                                    CAN_STATE_THREAD_PRIORITY,
                                    0,
                                    K_NO_WAIT);
    if (!can_state_tid)
    {
        printf("ERROR spawning can state thread\n");
    }

    rx_tid = k_thread_create(&rx_thread_data,
                             rx_thread_stack,
                             K_THREAD_STACK_SIZEOF(rx_thread_stack),
                             rx_thread,
                             NULL,
                             NULL,
                             NULL,
                             RX_THREAD_PRIORITY,
                             0,
                             K_NO_WAIT);
    if (!rx_tid)
    {
        printf("ERROR spawning rx thread\n");
    }

    tx_tid = k_thread_create(&tx_thread_data,
                             tx_thread_stack,
                             K_THREAD_STACK_SIZEOF(tx_thread_stack),
                             tx_thread,
                             NULL,
                             NULL,
                             NULL,
                             TX_THREAD_PRIORITY,
                             0,
                             K_NO_WAIT);
    if (!tx_tid)
    {
        printf("ERROR spawning tx thread\n");
    }

    can_set_state_change_callback(can_dev, state_change_callback, &state_change_work);
}
