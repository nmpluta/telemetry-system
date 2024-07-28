/*
 * Copyright (c) 2024 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_mqtt_publisher_sample, LOG_LEVEL_DBG);

#include "mqtt_threads.h"
#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>

#include <errno.h>
#include <string.h>

#include "mqtt_config.h"

#if defined(CONFIG_USERSPACE)
#include <zephyr/app_memory/app_memdomain.h>
K_APPMEM_PARTITION_DEFINE(app_partition);
struct k_mem_domain app_domain;
#define MQTT_BMEM K_MQTT_BMEM(app_partition)
#define MQTT_DMEM K_MQTT_DMEM(app_partition)
#else
#define MQTT_BMEM
#define MQTT_DMEM
#endif

/* MQTT thread configuration */
#define MQTT_THREAD_SLEEP_MS   K_MSEC(5000)
#define MQTT_THREAD_STACK_SIZE 4096
#define MQTT_THREAD_PRIORITY   1

K_THREAD_STACK_DEFINE(mqtt_stack, MQTT_THREAD_STACK_SIZE);

struct k_thread mqtt_thread_data;
k_tid_t         mqtt_tid;

/* Buffers for MQTT client. */
static MQTT_BMEM uint8_t rx_buffer[MQTT_MQTT_BUFFER_SIZE];
static MQTT_BMEM uint8_t tx_buffer[MQTT_MQTT_BUFFER_SIZE];

#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
/* Making RX buffer large enough that the full IPv6 packet can fit into it */
#define MQTT_LIB_WEBSOCKET_RECV_BUF_LEN 1280

/* Websocket needs temporary buffer to store partial packets */
static MQTT_BMEM uint8_t temp_ws_rx_buf[MQTT_LIB_WEBSOCKET_RECV_BUF_LEN];
#endif

/* The mqtt client struct */
static MQTT_BMEM struct mqtt_client client_ctx;

/* MQTT Broker details. */
static MQTT_BMEM struct sockaddr_storage broker;

#if defined(CONFIG_SOCKS)
static MQTT_BMEM struct sockaddr socks5_proxy;
#endif

static MQTT_BMEM struct pollfd fds[1];
static MQTT_BMEM int           nfds;

static MQTT_BMEM bool connected;

#if defined(CONFIG_MQTT_LIB_TLS)

#include "test_certs.h"

#define TLS_SNI_HOSTNAME "localhost"
#define MQTT_CA_CERT_TAG 1
#define MQTT_PSK_TAG     2

static MQTT_DMEM sec_tag_t m_sec_tags[] = {
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
    MQTT_CA_CERT_TAG,
#endif
#if defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
    MQTT_PSK_TAG,
#endif
};

static int tls_init(void)
{
    int err = -EINVAL;

#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
    err = tls_credential_add(MQTT_CA_CERT_TAG,
                             TLS_CREDENTIAL_CA_CERTIFICATE,
                             ca_certificate,
                             sizeof(ca_certificate));
    if (err < 0)
    {
        LOG_ERR("Failed to register public certificate: %d", err);
        return err;
    }
#endif

#if defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
    err = tls_credential_add(
        MQTT_PSK_TAG, TLS_CREDENTIAL_PSK, client_psk, sizeof(client_psk));
    if (err < 0)
    {
        LOG_ERR("Failed to register PSK: %d", err);
        return err;
    }

    err = tls_credential_add(MQTT_PSK_TAG,
                             TLS_CREDENTIAL_PSK_ID,
                             client_psk_id,
                             sizeof(client_psk_id) - 1);
    if (err < 0)
    {
        LOG_ERR("Failed to register PSK ID: %d", err);
    }
#endif

    return err;
}

#endif /* CONFIG_MQTT_LIB_TLS */

static void prepare_fds(struct mqtt_client *client)
{
    if (client->transport.type == MQTT_TRANSPORT_NON_SECURE)
    {
        fds[0].fd = client->transport.tcp.sock;
    }
#if defined(CONFIG_MQTT_LIB_TLS)
    else if (client->transport.type == MQTT_TRANSPORT_SECURE)
    {
        fds[0].fd = client->transport.tls.sock;
    }
#endif

    fds[0].events = POLLIN;
    nfds          = 1;
}

static void clear_fds(void)
{
    nfds = 0;
}

static int wait(int timeout)
{
    int ret = 0;

    if (nfds > 0)
    {
        ret = poll(fds, nfds, timeout);
        if (ret < 0)
        {
            LOG_ERR("poll error: %d", errno);
        }
    }

    return ret;
}

void mqtt_evt_handler(struct mqtt_client *const client,
                      const struct mqtt_evt    *evt)
{
    int err;

    switch (evt->type)
    {
        case MQTT_EVT_CONNACK:
            if (evt->result != 0)
            {
                LOG_ERR("MQTT connect failed %d", evt->result);
                break;
            }

            connected = true;
            LOG_INF("MQTT client connected!");

            break;

        case MQTT_EVT_DISCONNECT:
            LOG_INF("MQTT client disconnected %d", evt->result);

            connected = false;
            clear_fds();

            break;

        case MQTT_EVT_PUBACK:
            if (evt->result != 0)
            {
                LOG_ERR("MQTT PUBACK error %d", evt->result);
                break;
            }

            LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

            break;

        case MQTT_EVT_PUBREC:
            if (evt->result != 0)
            {
                LOG_ERR("MQTT PUBREC error %d", evt->result);
                break;
            }

            LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

            const struct mqtt_pubrel_param rel_param
                = { .message_id = evt->param.pubrec.message_id };

            err = mqtt_publish_qos2_release(client, &rel_param);
            if (err != 0)
            {
                LOG_ERR("Failed to send MQTT PUBREL: %d", err);
            }

            break;

        case MQTT_EVT_PUBCOMP:
            if (evt->result != 0)
            {
                LOG_ERR("MQTT PUBCOMP error %d", evt->result);
                break;
            }

            LOG_INF("PUBCOMP packet id: %u", evt->param.pubcomp.message_id);

            break;

        case MQTT_EVT_PINGRESP:
            LOG_INF("PINGRESP packet");
            break;

        default:
            break;
    }
}

static char *get_mqtt_payload(enum mqtt_qos qos)
{
#if MQTT_BLUEMIX_TOPIC
    static MQTT_BMEM char payload[30];

    snprintk(payload, sizeof(payload), "{d:{temperature:%d}}", sys_rand8_get());
#else
    static MQTT_DMEM char payload[] = "DOORS:OPEN_QoSx";

    payload[strlen(payload) - 1] = '0' + qos;
#endif

    return payload;
}

static char *get_mqtt_topic(void)
{
#if MQTT_BLUEMIX_TOPIC
    return "iot-2/type/" BLUEMIX_DEVTYPE "/id/" BLUEMIX_DEVID
           "/evt/" BLUEMIX_EVENT "/fmt/" BLUEMIX_FORMAT;
#else
    return "sensors";
#endif
}

static int publish(struct mqtt_client *client, enum mqtt_qos qos)
{
    struct mqtt_publish_param param;

    param.message.topic.qos        = qos;
    param.message.topic.topic.utf8 = (uint8_t *)get_mqtt_topic();
    param.message.topic.topic.size = strlen(param.message.topic.topic.utf8);
    param.message.payload.data     = get_mqtt_payload(qos);
    param.message.payload.len      = strlen(param.message.payload.data);
    param.message_id               = sys_rand16_get();
    param.dup_flag                 = 0U;
    param.retain_flag              = 0U;

    return mqtt_publish(client, &param);
}

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))

static void broker_init(void)
{
#if defined(CONFIG_NET_IPV6)
    struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

    broker6->sin6_family = AF_INET6;
    broker6->sin6_port   = htons(SERVER_PORT);
    inet_pton(AF_INET6, SERVER_ADDR, &broker6->sin6_addr);

#if defined(CONFIG_SOCKS)
    struct sockaddr_in6 *proxy6 = (struct sockaddr_in6 *)&socks5_proxy;

    proxy6->sin6_family = AF_INET6;
    proxy6->sin6_port   = htons(SOCKS5_PROXY_PORT);
    inet_pton(AF_INET6, SOCKS5_PROXY_ADDR, &proxy6->sin6_addr);
#endif
#else
    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

    broker4->sin_family = AF_INET;
    broker4->sin_port   = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);
#if defined(CONFIG_SOCKS)
    struct sockaddr_in *proxy4 = (struct sockaddr_in *)&socks5_proxy;

    proxy4->sin_family = AF_INET;
    proxy4->sin_port   = htons(SOCKS5_PROXY_PORT);
    inet_pton(AF_INET, SOCKS5_PROXY_ADDR, &proxy4->sin_addr);
#endif
#endif
    // Print broker server address
    LOG_INF("Broker server address: %s", SERVER_ADDR);
}

static void client_init(struct mqtt_client *client)
{
    mqtt_client_init(client);

    broker_init();

    /* MQTT client configuration */
    client->broker           = &broker;
    client->evt_cb           = mqtt_evt_handler;
    client->client_id.utf8   = (uint8_t *)MQTT_CLIENTID;
    client->client_id.size   = strlen(MQTT_CLIENTID);
    client->password         = NULL;
    client->user_name        = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    /* MQTT buffers configuration */
    client->rx_buf      = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf      = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    /* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
    client->transport.type = MQTT_TRANSPORT_SECURE_WEBSOCKET;
#else
    client->transport.type = MQTT_TRANSPORT_SECURE;
#endif

    struct mqtt_sec_config *tls_config = &client->transport.tls.config;

    tls_config->peer_verify   = TLS_PEER_VERIFY_REQUIRED;
    tls_config->cipher_list   = NULL;
    tls_config->sec_tag_list  = m_sec_tags;
    tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
    tls_config->hostname = TLS_SNI_HOSTNAME;
#else
    tls_config->hostname = NULL;
#endif

#else
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
    client->transport.type = MQTT_TRANSPORT_NON_SECURE_WEBSOCKET;
#else
    client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
#endif

#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
    client->transport.websocket.config.host        = SERVER_ADDR;
    client->transport.websocket.config.url         = "/mqtt";
    client->transport.websocket.config.tmp_buf     = temp_ws_rx_buf;
    client->transport.websocket.config.tmp_buf_len = sizeof(temp_ws_rx_buf);
    client->transport.websocket.timeout            = 5 * MSEC_PER_SEC;
#endif

#if defined(CONFIG_SOCKS)
    mqtt_client_set_proxy(client,
                          &socks5_proxy,
                          socks5_proxy.sa_family == AF_INET
                              ? sizeof(struct sockaddr_in)
                              : sizeof(struct sockaddr_in6));
#endif
}

/* In this routine we block until the connected variable is 1 */
static int try_to_connect(struct mqtt_client *client)
{
    int rc, i = 0;

    while (i++ < MQTT_CONNECT_TRIES && !connected)
    {

        client_init(client);

        rc = mqtt_connect(client);
        if (rc != 0)
        {
            PRINT_RESULT("mqtt_connect", rc);
            k_sleep(K_MSEC(MQTT_SLEEP_MSECS));
            continue;
        }

        prepare_fds(client);

        if (wait(MQTT_CONNECT_TIMEOUT_MS))
        {
            mqtt_input(client);
        }

        if (!connected)
        {
            mqtt_abort(client);
        }
    }

    if (connected)
    {
        return 0;
    }

    return -EINVAL;
}

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
    int64_t remaining  = timeout;
    int64_t start_time = k_uptime_get();
    int     rc;

    while (remaining > 0 && connected)
    {
        if (wait(remaining))
        {
            rc = mqtt_input(client);
            if (rc != 0)
            {
                PRINT_RESULT("mqtt_input", rc);
                return rc;
            }
        }

        rc = mqtt_live(client);
        if (rc != 0 && rc != -EAGAIN)
        {
            PRINT_RESULT("mqtt_live", rc);
            return rc;
        }
        else if (rc == 0)
        {
            rc = mqtt_input(client);
            if (rc != 0)
            {
                PRINT_RESULT("mqtt_input", rc);
                return rc;
            }
        }

        remaining = timeout + start_time - k_uptime_get();
    }

    return 0;
}

#define SUCCESS_OR_EXIT(rc) \
    {                       \
        if (rc != 0)        \
        {                   \
            return 1;       \
        }                   \
    }
#define SUCCESS_OR_BREAK(rc) \
    {                        \
        if (rc != 0)         \
        {                    \
            break;           \
        }                    \
    }

static int publisher(void)
{
    int i, rc, r = 0;

    LOG_INF("attempting to connect: ");
    rc = try_to_connect(&client_ctx);
    PRINT_RESULT("try_to_connect", rc);
    SUCCESS_OR_EXIT(rc);

    i = 0;
    while (i++ < CONFIG_MQTT_MAX_ITERATIONS && connected)
    {
        r = -1;

        rc = mqtt_ping(&client_ctx);
        PRINT_RESULT("mqtt_ping", rc);
        SUCCESS_OR_BREAK(rc);

        rc = process_mqtt_and_sleep(&client_ctx, MQTT_SLEEP_MSECS);
        SUCCESS_OR_BREAK(rc);

        rc = publish(&client_ctx, MQTT_QOS_0_AT_MOST_ONCE);
        PRINT_RESULT("mqtt_publish", rc);
        SUCCESS_OR_BREAK(rc);

        rc = process_mqtt_and_sleep(&client_ctx, MQTT_SLEEP_MSECS);
        SUCCESS_OR_BREAK(rc);

        rc = publish(&client_ctx, MQTT_QOS_1_AT_LEAST_ONCE);
        PRINT_RESULT("mqtt_publish", rc);
        SUCCESS_OR_BREAK(rc);

        rc = process_mqtt_and_sleep(&client_ctx, MQTT_SLEEP_MSECS);
        SUCCESS_OR_BREAK(rc);

        rc = publish(&client_ctx, MQTT_QOS_2_EXACTLY_ONCE);
        PRINT_RESULT("mqtt_publish", rc);
        SUCCESS_OR_BREAK(rc);

        rc = process_mqtt_and_sleep(&client_ctx, MQTT_SLEEP_MSECS);
        SUCCESS_OR_BREAK(rc);

        r = 0;
    }

    rc = mqtt_disconnect(&client_ctx);
    PRINT_RESULT("mqtt_disconnect", rc);

    LOG_INF("Bye!");

    return r;
}

void mqtt_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Initialization of mqtt_thread.");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    int r = 0, i = 0;

    while (!CONFIG_MQTT_MAX_CONNECTIONS || i++ < CONFIG_MQTT_MAX_CONNECTIONS)
    {
        r = publisher();

        if (!CONFIG_MQTT_MAX_CONNECTIONS)
        {
            k_sleep(MQTT_THREAD_SLEEP_MS);
        }
    }

    return;
}

void mqtt_threads_init(void)
{
    mqtt_tid = k_thread_create(&mqtt_thread_data,
                               mqtt_stack,
                               K_THREAD_STACK_SIZEOF(mqtt_stack),
                               mqtt_thread,
                               NULL,
                               NULL,
                               NULL,
                               MQTT_THREAD_PRIORITY,
                               0,
                               K_NO_WAIT);

    if (!mqtt_tid)
    {
        LOG_ERR("Failed to create mqtt_thread");
    }
}
