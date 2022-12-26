/*
 * Copyright (c) 2022 Natalia Pluta
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

#define MAX_OBD2_RES_ID        0x7EF
#define MIN_OBD2_RES_ID        0x7E8
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
 * Iterate through all the bits of the binary number and increment the count variable if the current
 * bit is 1.
 *
 * @param binary_number The binary number whose number of 1's is to be counted.
 *
 * @return The number of 1's in the binary number.
 */
uint8_t binary_ones_count(uint32_t binary_number)
{
    // Initialise count variables
    uint8_t ones_number = 0;

    // Iterate through all the bits
    while (binary_number > 0)
    {
        // If current bit is 1
        if (binary_number & 1)
        {
            ones_number++;
        }
        binary_number = binary_number >> 1;
    }
    return ones_number;
}

/**
 * It prints the contents of a CAN frame to the console
 *
 * @param frame The CAN frame to print
 */
void obd2_frame_print(struct can_frame * frame)
{
    LOG_INF("[OBD2] [%03x] %02x %02x %02x %02x %02x %02x %02x %02x",
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
 * It prints the supported PIDs for each service
 */
void obd2_acq_supported_pids_print(void)
{
    uint8_t pid_index = 0;

    LOG_INF("[OBD2] Supported CURRENT_DATA service PIDs:");
    for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
    {
        LOG_INF("Supported pid 0x%02x-0x%02x: 0x%08x.",
               pid_index * 0x20 + 1,
               (pid_index + 1) * 0x20,
               m_current_data_supported_pids[pid_index].acq_pids);
    }

    LOG_INF("[OBD2] Supported CURRENT_DATA service PIDs:");
    for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
    {
        LOG_INF("Supported pid 0x%02x-0x%02x: 0x%08x.",
               pid_index * 0x20 + 1,
               (pid_index + 1) * 0x20,
               m_freeze_data_supported_pids[pid_index].acq_pids);
    }

    LOG_INF("[OBD2] Supported TEST_RESULTS_NON_CAN service PIDs:");
    for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
    {
        LOG_INF("Supported pid 0x%02x-0x%02x: 0x%08x.",
               pid_index * 0x20 + 1,
               (pid_index + 1) * 0x20,
               m_test_res_non_can_supported_pids[pid_index].acq_pids);
    }

    LOG_INF("[OBD2] Supported VEHICLE_INFORMATION service PIDs:");
    for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
    {
        LOG_INF("Supported pid 0x%02x-0x%02x: 0x%08x.",
               pid_index * 0x20 + 1,
               (pid_index + 1) * 0x20,
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
 * It takes a CAN frame and returns the data portion of the frame as a 32-bit unsigned integer
 *
 * @param frame The CAN frame to get the data from.
 *
 * @return The data from the CAN frame.
 */
uint32_t obd2_frame_data_get(struct can_frame * frame)
{
    return ((uint32_t)frame->data[3]) + ((uint32_t)frame->data[4] << 8) +
           ((uint32_t)frame->data[5] << 16) + ((uint32_t)frame->data[6] << 24);
}

/**
 * It takes a CAN frame and checks the PIDs that are supported by the vehicle
 *
 * @param frame The CAN frame that was received.
 */
void obd2_supported_pids_get(struct can_frame * frame)
{
    uint8_t pid_index = 0;
    switch (frame->data[1] - SERVICE_RESPONE_OFFSET)
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

/**
 * It waits for the supported PIDs to be received, then it prints them and waits for the next CAN
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
                                      .id    = MAX_OBD2_RES_ID,
                                      .mask  = !(MAX_OBD2_RES_ID ^ MIN_OBD2_RES_ID)};

    struct can_frame frame;
    int              filter_id;

    filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
    LOG_INF("Filter id: %d.", filter_id);

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
        while (k_msgq_put(&sd_msgq, &frame, K_NO_WAIT) != 0)
        {
            /* message queue is full: purge old data & try again */
            k_msgq_purge(&sd_msgq);
        }

        LOG_INF("[OBD2] RESPOND:");
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
        LOG_INF("Sending failed [%d].", ret);
    }
    else
    {
        LOG_INF("[OBD2] REQUEST: Service = %02x Pid = %02x.", service, pid);
    }
    return ret;
}

/**
 * It sends a request for each PID in the supported PIDs list for the given service
 *
 * @param service The service to send the request to.
 */
void obd2_supported_pids_requests_send(obd2_services_t service)
{
    uint8_t pid_index = 0;
    switch (service)
    {
        case CURRENT_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_current_data_supported_pids[pid_index].req_pid);
                k_sleep(SLEEP_TIME);
            }
            break;

        case FREEZE_FRAME_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_freeze_data_supported_pids[pid_index].req_pid);
                k_sleep(SLEEP_TIME);
            }
            break;

        case TEST_RESULTS_NON_CAN:
            for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_test_res_non_can_supported_pids[pid_index].req_pid);
                k_sleep(SLEEP_TIME);
            }
            break;

        case VEHICLE_INFORMATION:
            for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
            {
                obd2_request_send(service, m_vehicle_info_supported_pids[pid_index].req_pid);
                k_sleep(SLEEP_TIME);
            }
            break;

        default:
            LOG_INF("Service %02x does not have PID that shows supported PIDs.", service);
            break;
    }
}

/**
 * It counts the number of ones in the binary representation of the PIDs supported by the given
 * service
 *
 * @param service The service to get the number of supported PIDs for.
 *
 * @return The number of supported PIDs for the given service.
 */
uint8_t obd2_supported_pids_number_get(obd2_services_t service)
{
    uint8_t supported_pids_number = 0;
    uint8_t pid_index;

    switch (service)
    {
        case CURRENT_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                supported_pids_number +=
                    binary_ones_count(m_current_data_supported_pids[pid_index].acq_pids);
            }
            break;

        case FREEZE_FRAME_DATA:
            for (pid_index = 0; pid_index < DATA_SUPPORTED_PIDS; pid_index++)
            {
                supported_pids_number +=
                    binary_ones_count(m_freeze_data_supported_pids[pid_index].acq_pids);
            }
            break;

        case TEST_RESULTS_NON_CAN:
            for (pid_index = 0; pid_index < TEST_RES_NON_CAN_SUPPORTED_PIDS; pid_index++)
            {
                supported_pids_number +=
                    binary_ones_count(m_test_res_non_can_supported_pids[pid_index].acq_pids);
            }
            break;

        case VEHICLE_INFORMATION:
            for (pid_index = 0; pid_index < VEHICLE_INFO_SUPPORTED_PIDS; pid_index++)
            {
                supported_pids_number +=
                    binary_ones_count(m_vehicle_info_supported_pids[pid_index].acq_pids);
            }
            break;

        default:
            LOG_INF("Service %02x does not have PID that shows supported PIDs.", service);
            break;
    }

    return supported_pids_number;
}

/**
 * It takes a pointer to an array of PIDs, and a service, and it fills the array with the PIDs that
 * are supported by the service
 *
 * @param p_service_pids pointer to the array where the supported PIDs will be stored.
 * @param service The service to decode the supported PIDs for.
 */
void obd2_supported_pids_decode(uint8_t * p_service_pids, obd2_services_t service)
{
    uint8_t                 pid_offset;
    uint8_t                 pid;
    uint32_t                pids_to_decode;
    uint8_t                 supported_pids_group_num = 0;
    obd2_supported_pids_t * p_service_supported_pids = NULL;

    switch (service)
    {
        case CURRENT_DATA:
            supported_pids_group_num = DATA_SUPPORTED_PIDS;
            p_service_supported_pids = m_current_data_supported_pids;
            break;

        case FREEZE_FRAME_DATA:
            supported_pids_group_num = DATA_SUPPORTED_PIDS;
            p_service_supported_pids = m_freeze_data_supported_pids;
            break;

        case TEST_RESULTS_NON_CAN:
            supported_pids_group_num = TEST_RES_NON_CAN_SUPPORTED_PIDS;
            p_service_supported_pids = m_test_res_non_can_supported_pids;
            break;

        case VEHICLE_INFORMATION:
            supported_pids_group_num = VEHICLE_INFO_SUPPORTED_PIDS;
            p_service_supported_pids = m_vehicle_info_supported_pids;
            break;

        default:
            LOG_INF("Service %02x does not have PID that shows supported PIDs.", service);
            break;
    }

    for (uint8_t pid_group = 0; pid_group < supported_pids_group_num; pid_group++)
    {
        pid            = 0;
        pid_offset     = p_service_supported_pids[pid_group].req_pid + 1;
        pids_to_decode = p_service_supported_pids[pid_group].acq_pids;
        while (pids_to_decode > 0)
        {
            // If current pid is supported
            if (pids_to_decode & 1)
            {
                *p_service_pids = pid + pid_offset;
                p_service_pids++;
            }
            pid++;
            pids_to_decode = pids_to_decode >> 1;
        }
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

    uint8_t vehicle_info_pids_num = obd2_supported_pids_number_get(VEHICLE_INFORMATION);
    uint8_t vehicle_info_pids[vehicle_info_pids_num];
    obd2_supported_pids_decode(vehicle_info_pids, VEHICLE_INFORMATION);

    obd2_services_t service = VEHICLE_INFORMATION;
    uint8_t         pid_idx;

    for (pid_idx = 0; pid_idx < vehicle_info_pids_num; pid_idx++)
    {
        obd2_request_send(service, vehicle_info_pids[pid_idx]);
        k_sleep(SLEEP_TIME);
    }

    uint8_t current_data_pids_num = obd2_supported_pids_number_get(CURRENT_DATA);
    uint8_t current_data_pids[current_data_pids_num];
    obd2_supported_pids_decode(current_data_pids, CURRENT_DATA);

    service = CURRENT_DATA;
    pid_idx = 0;

    while (1)
    {
        obd2_request_send(service, current_data_pids[pid_idx]);
        pid_idx++;
        if (pid_idx == current_data_pids_num)
        {
            pid_idx = 0;
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
