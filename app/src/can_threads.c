/*
 * Copyright (c) 2022-2023 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file can_threads.c
 * @brief Can related threads
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_threads, LOG_LEVEL_DBG);

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#include "can_threads.h"

#define SLEEP_TIME K_MSEC(200)

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY   2

#define TX_THREAD_STACK_SIZE 512
#define TX_THREAD_PRIORITY   2

#define CAN_STATE_THREAD_STACK_SIZE 512
#define CAN_STATE_THREAD_PRIORITY   2

#define ALL_CAN_MSG_MASK 0x00
#define ALL_CAN_MSG_ID   0x00

#define SERVICE_RESPONE_OFFSET 0x40

#define BITRATE_500K        500000
#define SAMPLING_POINT_87_5 875

#define DATA_SUPPORTED_PIDS             6
#define TEST_RES_NON_CAN_SUPPORTED_PIDS 1
#define VEHICLE_INFO_SUPPORTED_PIDS     1
#define REQUEST_SUPPORTED_PIDS_ATTEMPTS 3

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_state_stack, CAN_STATE_THREAD_STACK_SIZE);

struct k_thread rx_thread_data;
struct k_thread tx_thread_data;
struct k_thread can_state_thread_data;
struct k_work   state_change_work;
struct k_mutex  supp_pids_finished_mutex;

k_tid_t rx_tid;
k_tid_t tx_tid;
k_tid_t can_state_tid;

const struct device * const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

enum can_state         current_state;
struct can_bus_err_cnt current_err_cnt;

bool m_rx_supported_pids_finished = false;
bool m_tx_supported_pids_finished = false;

CAN_MSGQ_DEFINE(can_msgq, 10);
extern struct k_msgq sd_msgq;

/**
 * It prints the contents of a CAN frame to the console
 *
 * @param frame The CAN frame to print
 */
void can_frame_print(struct can_frame * frame)
{
    LOG_INF("[CAN] [%03x] %02x %02x %02x %02x %02x %02x %02x %02x",
           frame->id,
           frame->data[0],
           frame->data[1],
           frame->data[2],
           frame->data[3],
           frame->data[4],
           frame->data[5],
           frame->data[6],
           frame->data[7]);
}

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
        LOG_INF("Callback! error-code: %d\nSender: %s", error, sender);
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

/**
 * It waits for any meesage to be received, then it prints them and waits for the next CAN
 * frame
 *
 * @param arg1 CAN device
 * @param arg2 CAN device
 * @param arg3 The third argument to the thread.
 */
void rx_thread(void * arg1, void * arg2, void * arg3)
{
    LOG_INF("Initialization of rx_thread.");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    const struct can_filter filter = {.flags = CAN_FILTER_DATA,
                                      .id    = ALL_CAN_MSG_ID,
                                      .mask  = ALL_CAN_MSG_MASK};

    struct can_frame frame;
    int              filter_id;

    filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
    LOG_INF("Filter id: %d.", filter_id);

    int ret;
    (void)ret;

    while (1)
    {
        k_msgq_get(&can_msgq, &frame, K_FOREVER);
        while (k_msgq_put(&sd_msgq, &frame, K_NO_WAIT) != 0)
        {
            /* message queue is full: purge old data & try again */
            k_msgq_purge(&sd_msgq);
        }

        LOG_INF("Received message id = [%d].", frame.id);
        can_frame_print(&frame);
        k_sleep(SLEEP_TIME);
    }
}

/**
 * It sends requests for supported PIDs, waits for the responses, and then sends requests for the
 * supported PIDs
 *
 * @param arg1 Pointer to the first argument passed to the thread.
 * @param arg2 The CAN interface to use.
 * @param arg3 The third argument to the thread.
 */
void tx_thread(void * arg1, void * arg2, void * arg3)
{
    LOG_INF("Initialization of tx_thread.");

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
            LOG_INF("Sending failed [%d].", ret);
        }
        else
        {
            LOG_INF("Message id = [%d] successfully sended.\n", frame.id);
        }

        k_sleep(SLEEP_TIME);
    }
}

/**
 * It prints the CAN controller state, the number of received and transmitted errors
 *
 * @param unused1 The first parameter of the thread function.
 * @param unused2 The second parameter to the thread function.
 * @param unused3 This is the third parameter passed to the thread.
 */
void can_state_thread(void * unused1, void * unused2, void * unused3)
{
    LOG_INF("Initialization of can_state_thread.");

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
            LOG_INF("Failed to get CAN controller state: %d", err);
            k_sleep(K_MSEC(100));
            continue;
        }

        if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
            err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt || state_prev != state)
        {

            err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
            err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
            state_prev              = state;
            LOG_INF("state: %s"
                   "rx error count: %d"
                   "tx error count: %d",
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

/**
 * It prints the current state of the CAN controller.
 * If the controller is in the bus-off state and CONFIG_CAN_AUTO_BUS_OFF_RECOVERY=y, it
 * attempts to recover from it.
 *
 * @param work The work item to be executed.
 */
void state_change_work_handler(struct k_work * work)
{
    LOG_INF("State Change ISR\nstate: %s"
           "rx error count: %d"
           "tx error count: %d",
           state_to_str(current_state),
           current_err_cnt.rx_err_cnt,
           current_err_cnt.tx_err_cnt);

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    if (current_state == CAN_STATE_BUS_OFF)
    {
        LOG_INF("Recover from bus-off");

        if (can_recover(can_dev, K_MSEC(100)) != 0)
        {
            LOG_INF("Recovery timed out");
        }
    }
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
}

/**
 * It's a callback function that gets called when the CAN state changes
 *
 * @param dev The device that triggered the callback.
 * @param state The current state of the CAN controller.
 * @param err_cnt A struct containing the current error counters for the CAN bus.
 * @param user_data A pointer to the work object that will be submitted to the work queue.
 */
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
        LOG_INF("CAN: Device %s not ready.", can_dev->name);
    }

    ret = can_calc_timing(can_dev, &timing, BITRATE_500K, SAMPLING_POINT_87_5);
    if (ret > 0)
    {
        LOG_INF("Sample-Point error: %d", ret);
    }

    if (ret < 0)
    {
        LOG_INF("Failed to calc a valid timing");
        return;
    }

    ret = can_set_timing(can_dev, &timing);
    if (0 != ret)
    {
        LOG_INF("Failed to set timing");
    }

    ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
    if (0 != ret)
    {
        LOG_INF("Failed to set CAN controller operation service");
    }

    ret = can_start(can_dev);
    if (0 != ret)
    {
        LOG_INF("Failed to start CAN controller");
    }

    LOG_INF("CAN: Device %s initialization finished.", can_dev->name);
}

/**
 * It creates three threads, one for each of the three main tasks of the CAN driver:
 *
 * * `can_state_thread`: This thread is responsible for monitoring the state of the CAN driver and
 * changing it as necessary.
 * * `rx_thread`: This thread is responsible for receiving CAN messages.
 * * `tx_thread`: This thread is responsible for transmitting CAN messages
 */
void can_threads_init(void)
{
    k_work_init(&state_change_work, state_change_work_handler);
    k_mutex_init(&supp_pids_finished_mutex);

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
        LOG_ERR("ERROR spawning can state thread.");
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
        LOG_ERR("ERROR spawning rx thread");
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
        LOG_ERR("ERROR spawning tx thread.");
    }

    can_set_state_change_callback(can_dev, state_change_callback, &state_change_work);
}
