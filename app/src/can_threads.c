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
#include <zephyr/logging/log.h>

#include "can_threads.h"

#define CAN_STATE_THREAD_STACK_SIZE 512
#define CAN_STATE_THREAD_PRIORITY   2
#define LED_MSG_ID                  0x10
#define COUNTER_MSG_ID              0x12345
#define SLEEP_TIME                  K_MSEC(10)

#define MAX_PID_SERVICE_1 0xC4

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY   2

#define TX_THREAD_STACK_SIZE 512
#define TX_THREAD_PRIORITY   2

#define CAN_STATE_THREAD_STACK_SIZE 512
#define CAN_STATE_THREAD_PRIORITY   2

#define MAX_OBD2_RES_ID 0x7EF
#define MIN_OBD2_RES_ID 0x7E8

#define VEHICLE_SPEED_PID 13

#define BITRATE_500K        500000
#define SAMPLING_POINT_87_5 875

#define DATA_SUPPORTED_PIDS             6
#define TEST_RES_NON_CAN_SUPPORTED_PIDS 1
#define VEHICLE_INFO_SUPPORTED_PIDS     1
#define REQUEST_SUPPORTED_PIDS_ATTEMPTS 3

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_state_stack, CAN_STATE_THREAD_STACK_SIZE);

LOG_MODULE_DECLARE(can_threads, LOG_LEVEL_DBG);

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

CAN_MSGQ_DEFINE(can_msgq, 2);

typedef enum
{
    CURRENT_DATA = 0x1,
    FREEZE_FRAME_DATA,
    STORED_DTCS,
    CLEAR_DTCS,
    TEST_RESULTS_NON_CAN,
    TEST_RESULTS_CAN,
    PENDING_DTCS,
    CONTROL_OPERATION,
    VEHICLE_INFORMATION,
    PERMAMENT_DTCS,
} obd2_services_t;

typedef struct
{
    uint8_t  req_pid;
    uint32_t acq_pids;
} obd2_supported_pids_t;

obd2_supported_pids_t m_current_data_supported_pids[DATA_SUPPORTED_PIDS] = {
    [0].req_pid = 0x00,
    [1].req_pid = 0x20,
    [2].req_pid = 0x40,
    [3].req_pid = 0x60,
    [4].req_pid = 0x80,
    [5].req_pid = 0xA0,
};
obd2_supported_pids_t m_freeze_data_supported_pids[DATA_SUPPORTED_PIDS] = {
    [0].req_pid = 0x00,
    [1].req_pid = 0x20,
    [2].req_pid = 0x40,
    [3].req_pid = 0x60,
    [4].req_pid = 0x80,
    [5].req_pid = 0xA0,
};
obd2_supported_pids_t m_test_res_non_can_supported_pids[TEST_RES_NON_CAN_SUPPORTED_PIDS] = {
    [0].req_pid = 0x00,
};
obd2_supported_pids_t m_vehicle_info_supported_pids[VEHICLE_INFO_SUPPORTED_PIDS] = {
    [0].req_pid = 0x00,
};

/**
 * It prints the contents of a CAN frame to the console
 *
 * @param frame The CAN frame to print
 */
void obd2_frame_print(struct can_frame * frame)
{
    printf("[OBD2] [%03x] %02x %02x %02x %02x %02x %02x %02x %02x\n",
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

void obd2_acq_supported_pids_print(void)
{
    uint8_t pid_index = 0;

    printf("[OBD2] Supported CURRENT_DATA service PIDs:\n");
    for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
    {
        printf("Index: %d -> supported: 0x%x.\n",
               pid_index,
               m_current_data_supported_pids[pid_index].acq_pids);
    }

    printf("[OBD2] Supported CURRENT_DATA service PIDs:\n");
    for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
    {
        printf("Index: %d -> supported: 0x%x.\n",
               pid_index,
               m_freeze_data_supported_pids[pid_index].acq_pids);
    }

    printf("[OBD2] Supported TEST_RESULTS_NON_CAN service PIDs:\n");
    for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
    {
        printf("Index: %d -> supported: 0x%x.\n",
               pid_index,
               m_test_res_non_can_supported_pids[pid_index].acq_pids);
    }

    printf("[OBD2] Supported VEHICLE_INFORMATION service PIDs:\n");
    for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
    {
        printf("Index: %d -> supported: 0x%x.\n",
               pid_index,
               m_vehicle_info_supported_pids[pid_index].acq_pids);
    }
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

uint32_t obd2_frame_data_get(struct can_frame * frame)
{
    return (uint32_t)(frame->data[3] + frame->data[4] + frame->data[5] + frame->data[6]);
}

void obd2_supported_pids_get(struct can_frame * frame)
{
    uint8_t pid_index = 0;
    switch (frame->data[1] - 0x40)
    {
        case CURRENT_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                if (frame->data[2] == m_current_data_supported_pids[pid_index].req_pid)
                {
                    m_current_data_supported_pids[pid_index].acq_pids = obd2_frame_data_get(frame);
                }
            }
            break;
        case FREEZE_FRAME_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                if (frame->data[2] == m_freeze_data_supported_pids[pid_index].req_pid)
                {
                    m_freeze_data_supported_pids[pid_index].acq_pids = obd2_frame_data_get(frame);
                }
            }
            break;
        case TEST_RESULTS_NON_CAN:
            for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
            {
                if (frame->data[2] == m_test_res_non_can_supported_pids[pid_index].req_pid)
                {
                    m_test_res_non_can_supported_pids[pid_index].acq_pids =
                        obd2_frame_data_get(frame);
                }
            }
            break;
        case VEHICLE_INFORMATION:
            for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
            {
                if (frame->data[2] == m_vehicle_info_supported_pids[pid_index].req_pid)
                {
                    m_vehicle_info_supported_pids[pid_index].acq_pids = obd2_frame_data_get(frame);
                }
            }
            break;
        default:
            break;
    }
}

void rx_thread(void * arg1, void * arg2, void * arg3)
{
    printf("Initialization of rx_thread.\n");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    const struct can_filter filter = {.flags = CAN_FILTER_DATA,
                                      .id    = MAX_OBD2_RES_ID,
                                      .mask  = !(MAX_OBD2_RES_ID ^ MIN_OBD2_RES_ID)};

    struct can_frame frame;
    int              filter_id;

    filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
    printf("Filter id: %d.\n", filter_id);

    int ret;
    (void)ret;

    while (1)
    {
        k_mutex_lock(&supp_pids_finished_mutex, K_FOREVER);
        ret = k_msgq_get(&can_msgq, &frame, K_MSEC(1000));
        if (ret == -EAGAIN)
        {
            if (m_tx_supported_pids_finished)
            {
                m_rx_supported_pids_finished = true;
                k_mutex_unlock(&supp_pids_finished_mutex);
                break;
            }
            k_mutex_unlock(&supp_pids_finished_mutex);
            break;
        }
        else
        {
            obd2_supported_pids_get(&frame);
            k_mutex_unlock(&supp_pids_finished_mutex);
        }
    }

    k_mutex_lock(&supp_pids_finished_mutex, K_FOREVER);
    m_rx_supported_pids_finished = true;
    k_mutex_unlock(&supp_pids_finished_mutex);

    obd2_acq_supported_pids_print();

    while (1)
    {
        k_msgq_get(&can_msgq, &frame, K_FOREVER);

        printf("[OBD2] RESPOND:\n");
        obd2_frame_print(&frame);
        k_sleep(SLEEP_TIME);
    }
}

/**
 * It sends a request to the OBD2 device
 *
 * @param service The service to request.
 * @param pid The parameter ID.
 *
 * @return The return value of the function.
 */
int obd2_request_send(obd2_services_t service, uint8_t pid)
{
    int ret;
    (void)ret;

    struct can_frame frame = {
        .id   = 0x7DF,
        .dlc  = 8,
        .data = {0x2, service, pid, 0x0, 0x0, 0x0, 0x0, 0x0},
    };

    /* A variable that stores the return value of the function. */
    ret = can_send(can_dev, &frame, K_FOREVER, tx_irq_callback, NULL);
    if (ret != 0)
    {
        printf("Sending failed [%d].", ret);
    }
    else
    {
        printf("[OBD2] REQUEST: Service = %02x Pid = %02x.\n", service, pid);
    }
    return ret;
}

void obd2_supported_pids_requests_send(obd2_services_t service)
{
    uint8_t pid_index = 0;
    switch (service)
    {
        case CURRENT_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_current_data_supported_pids[pid_index].req_pid);
            }
            break;

        case FREEZE_FRAME_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_current_data_supported_pids[pid_index].req_pid);
            }
            break;

        case TEST_RESULTS_NON_CAN:
            for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_test_res_non_can_supported_pids[pid_index].req_pid);
            }
            break;

        case VEHICLE_INFORMATION:
            for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_vehicle_info_supported_pids[pid_index].req_pid);
            }
            break;

        default:
            printf("Service %02x does not have PID that shows supported PIDs.", service);
            break;
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

    for (uint8_t reqs_attempt = 0; reqs_attempt < REQUEST_SUPPORTED_PIDS_ATTEMPTS; reqs_attempt++)
    {
        obd2_supported_pids_requests_send(CURRENT_DATA);
        obd2_supported_pids_requests_send(VEHICLE_INFORMATION);
    }
    k_mutex_lock(&supp_pids_finished_mutex, K_FOREVER);
    m_tx_supported_pids_finished = true;
    k_mutex_unlock(&supp_pids_finished_mutex);

    while (1)
    {
        k_mutex_lock(&supp_pids_finished_mutex, K_FOREVER);
        if (m_rx_supported_pids_finished)
        {
            k_mutex_unlock(&supp_pids_finished_mutex);
            break;
        }
        k_mutex_unlock(&supp_pids_finished_mutex);
    }

    obd2_services_t service = CURRENT_DATA;
    uint8_t         pid     = 0;

    while (1)
    {
        obd2_request_send(service, pid);
        pid++;
        if (pid == MAX_PID_SERVICE_1)
        {
            pid = 0;
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

    ret = can_calc_timing(can_dev, &timing, BITRATE_500K, SAMPLING_POINT_87_5);
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
        printf("Failed to set CAN controller operation service");
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
