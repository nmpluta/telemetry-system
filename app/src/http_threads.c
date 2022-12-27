/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tagoio_http_post, LOG_LEVEL_DBG);

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>
#include <zephyr/random/rand32.h>
#include <zephyr/drivers/can.h>

#include "sockets.h"

#define HTTP_POST_THREAD_STACK_SIZE 4096
#define HTTP_POST_THREAD_PRIORITY   1
#define SLEEP_TIME                  K_MSEC(10)

K_THREAD_STACK_DEFINE(http_post_stack, HTTP_POST_THREAD_STACK_SIZE);
struct k_thread http_post_data;
k_tid_t         http_post_tid;

static struct tagoio_context ctx;

static void response_cb(struct http_response * rsp,
                        enum http_final_call   final_data,
                        void *                 user_data)
{
    if (final_data == HTTP_DATA_MORE)
    {
        LOG_DBG("Partial data received (%zd bytes)", rsp->data_len);
    }
    else if (final_data == HTTP_DATA_FINAL)
    {
        LOG_DBG("All the data received (%zd bytes)", rsp->data_len);
    }

    LOG_DBG("Response status %s", rsp->http_status);
}

static int collect_data(void)
{
#define lower 20000
#define upper 100000
#define base  1000.00f

    float temp;

    /* Generate a temperature between 20 and 100 celsius degree */
    temp = ((sys_rand32_get() % (upper - lower + 1)) + lower);
    temp /= base;

    (void)snprintf(ctx.payload,
                   sizeof(ctx.payload),
                   "{\"variable\": \"temperature\","
                   "\"unit\": \"c\",\"value\": %f}",
                   (double)temp);

    /* LOG doesn't print float #18351 */
    LOG_INF("Temp: %d", (int)temp);

    return 0;
}

static void next_turn(void)
{
    if (collect_data() < 0)
    {
        LOG_INF("Error collecting data.");
        return;
    }

    if (tagoio_connect(&ctx) < 0)
    {
        LOG_INF("No connection available.");
        return;
    }

    if (tagoio_http_push(&ctx, response_cb) < 0)
    {
        LOG_INF("Error pushing data.");
        return;
    }
}

/**
 * It will run forever, sleeping for a while, and then calling the next_turn() function
 *
 * @param arg1 The first argument to pass to the thread.
 * @param arg2 the name of the thread
 * @param arg3 The third argument to the thread.
 */
void http_post_thread(void * arg1, void * arg2, void * arg3)
{
    LOG_INF("Initialization of http_post_thread.");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    int ret;
    (void)ret;

    while (true)
    {
        next_turn();
        k_sleep(K_MSEC(CONFIG_TAGOIO_HTTP_PUSH_INTERVAL));
    }
}

/**
 * It creates a thread that will be used to send data to the server
 */
void http_threads_init(void)
{
    http_post_tid = k_thread_create(&http_post_data,
                                    http_post_stack,
                                    K_THREAD_STACK_SIZEOF(http_post_stack),
                                    http_post_thread,
                                    NULL,
                                    NULL,
                                    NULL,
                                    HTTP_POST_THREAD_PRIORITY,
                                    0,
                                    K_NO_WAIT);
    if (!http_post_tid)
    {
        LOG_ERR("Failed spawning http post thread.");
    }
}
