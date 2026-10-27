/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * lwip_utk_mqtt.c
 * MQTT client session for the Pico-Car telemetry link.
 *
 * Runs entirely inside the radio-owner task through lwip_utk_mqtt_poll(),
 * which is called on every radio poll.  It never blocks: each call looks at
 * the current state, does at most a few small pieces of work and returns.
 *
 *   IDLE --(link up + IP)--> CONNECTING --(accepted)--> CONNECTED
 *     ^                          |                          |
 *     |              (refused / 5 s / link down)       (lost/closed)
 *     |                          v                          v
 *     +---- link down ------ BACKOFF <----------------------+
 *
 * BACKOFF doubles its wait on every failure (1 s .. 5 s) and resets after
 * a successful connection, so a missing broker costs almost nothing.
 * CONNECTING has its own 5 s limit: lwIP would otherwise wait up to 100 s
 * for a broker that accepted TCP but never answered.
 *
 * What to publish comes from the comms module (comms_link.h); this file
 * only owns the session and moves messages between it and lwIP.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

#include "comms_link.h"
#include "lwip_utk_mqtt.h"
#include "mqtt_config.h"

#define MQTT_KEEP_ALIVE_S        10U
#define MQTT_RETRY_MIN_MS        1000U
#define MQTT_RETRY_MAX_MS        5000U  /* LAN broker: retries are cheap */
#define MQTT_CONNECT_TIMEOUT_MS  5000U  /* lwIP's own limit is 100 s     */
#define MQTT_SERVICE_MS          20U    /* how often to pull new messages */
#define MQTT_PUBLISH_BURST       4U     /* max publishes per service      */
#define MQTT_QOS_1               1U
#define MQTT_RETAIN              1U

#define TOPIC_STATUS             "robot/status"
#define TOPIC_COMMAND            "robot/command"
#define STATUS_ONLINE            "online"
#define STATUS_OFFLINE           "offline"

/* Statically allocated: no heap use for the client itself. */
static mqtt_client_t          g_client;
static lwip_utk_mqtt_status_t g_status;
static ip_addr_t              g_broker;
static bool                   g_b_broker_valid;
static bool                   g_b_initialized;
static uint32_t               g_state_since_ms;
static uint32_t               g_last_service_ms;
static bool                   g_b_announce_pending;
static bool                   g_b_subscribe_pending;
static bool                   g_b_net_was_ready;

/* A message taken from the comms module that lwIP could not accept yet
   (output buffer full).  Retried before anything new is taken. */
static comms_link_msg_t       g_pending;
static bool                   g_b_pending;

/* Incoming command being assembled; lwIP may deliver it in fragments. */
static bool                   g_b_cmd_is_ours;
static uint32_t               g_cmd_fill;
static uint8_t                g_cmd_buf[LWIP_UTK_MQTT_CMD_MAX];

static const struct mqtt_connect_client_info_t g_client_info =
{
    .client_id   = UTK_MQTT_CLIENT_ID,
    .client_user = NULL,
    .client_pass = NULL,
    .keep_alive  = MQTT_KEEP_ALIVE_S,
    .will_topic  = TOPIC_STATUS,
    .will_msg    = STATUS_OFFLINE,
    .will_qos    = MQTT_QOS_1,
    .will_retain = MQTT_RETAIN,
};


static void
enter_state (lwip_utk_mqtt_state_t new_state)
{
    g_status.state   = (uint32_t) new_state;
    g_state_since_ms = sys_now();
}


static void
start_backoff (int32_t reason)
{
    g_status.last_error = reason;
    comms_link_set_connected(false);

    if (0U == g_status.retry_delay_ms)
    {
        g_status.retry_delay_ms = MQTT_RETRY_MIN_MS;
    }
    else if (g_status.retry_delay_ms < MQTT_RETRY_MAX_MS)
    {
        g_status.retry_delay_ms *= 2U;

        if (g_status.retry_delay_ms > MQTT_RETRY_MAX_MS)
        {
            g_status.retry_delay_ms = MQTT_RETRY_MAX_MS;
        }
    }
    else
    {
        /* Already at the ceiling. */
    }

    enter_state(LWIP_UTK_MQTT_BACKOFF);
}


static void
incoming_publish_cb (void * p_arg, char const * p_topic, u32_t total_len)
{
    (void) p_arg;
    (void) total_len;

    g_b_cmd_is_ours = (0 == strcmp(p_topic, TOPIC_COMMAND));
    g_cmd_fill      = 0U;
}


static void
incoming_data_cb (void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    uint32_t room;
    uint32_t take;

    (void) p_arg;

    if (g_b_cmd_is_ours)
    {
        room = LWIP_UTK_MQTT_CMD_MAX - g_cmd_fill;
        take = (len < room) ? len : room;
        (void) memcpy(&g_cmd_buf[g_cmd_fill], p_data, take);
        g_cmd_fill += take;

        if (0U != (flags & MQTT_DATA_FLAG_LAST))
        {
            (void) memcpy(g_status.cmd_payload, g_cmd_buf, g_cmd_fill);
            g_status.cmd_len = g_cmd_fill;
            g_status.cmd_count++;
            g_b_cmd_is_ours = false;
            comms_link_on_command(g_cmd_buf, g_cmd_fill);
        }
    }
}


static void
connection_cb (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t conn_status)
{
    (void) p_client;
    (void) p_arg;

    if (MQTT_CONNECT_ACCEPTED == conn_status)
    {
        g_status.connects++;
        g_status.retry_delay_ms = 0U;
        g_status.last_error     = 0;
        g_b_announce_pending    = true;
        g_b_subscribe_pending   = true;
        comms_link_set_connected(true);
        enter_state(LWIP_UTK_MQTT_CONNECTED);
    }
    else
    {
        if (LWIP_UTK_MQTT_CONNECTED == g_status.state)
        {
            g_status.disconnects++;
        }

        start_backoff((int32_t) conn_status);
    }
}


static void
try_connect (void)
{
    err_t result;

    g_status.connect_attempts++;
    result = mqtt_client_connect(&g_client, &g_broker,
                                 (u16_t) UTK_MQTT_BROKER_PORT,
                                 connection_cb, NULL, &g_client_info);

    if (ERR_OK == result)
    {
        /* mqtt_client_connect() wipes the client, so callbacks go after. */
        mqtt_set_inpub_callback(&g_client, incoming_publish_cb,
                                incoming_data_cb, NULL);
        enter_state(LWIP_UTK_MQTT_CONNECTING);
    }
    else
    {
        start_backoff((int32_t) result);
    }
}


/* Publishes g_pending.  Returns false if lwIP is full and it must wait. */
static bool
publish_pending (void)
{
    err_t result;
    bool  b_sent = false;

    result = mqtt_publish(&g_client, g_pending.topic, g_pending.payload,
                          g_pending.payload_len, g_pending.qos,
                          g_pending.b_retain, NULL, NULL);

    if (ERR_OK == result)
    {
        g_status.published++;
        g_b_pending = false;
        b_sent      = true;
    }
    else if (ERR_MEM != result)
    {
        /* Not a temporary shortage: drop it rather than retry forever. */
        g_status.publish_errors++;
        g_status.last_error = (int32_t) result;
        g_b_pending = false;
    }
    else
    {
        /* lwIP buffer full: keep it and retry on the next service. */
    }

    return b_sent;
}


static void
service_connected (uint32_t now_ms)
{
    err_t    result;
    uint32_t burst;
    bool     b_more;

    if (g_b_announce_pending)
    {
        result = mqtt_publish(&g_client, TOPIC_STATUS, STATUS_ONLINE,
                              (u16_t) (sizeof(STATUS_ONLINE) - 1U),
                              MQTT_QOS_1, MQTT_RETAIN, NULL, NULL);
        g_b_announce_pending = (ERR_OK != result);   /* retry next poll */

        if (ERR_OK == result)
        {
            g_status.published++;
        }
    }

    if (g_b_subscribe_pending)
    {
        result = mqtt_sub_unsub(&g_client, TOPIC_COMMAND, MQTT_QOS_1,
                                NULL, NULL, 1U);
        g_b_subscribe_pending = (ERR_OK != result);
    }

    if ((now_ms - g_last_service_ms) >= MQTT_SERVICE_MS)
    {
        g_last_service_ms = now_ms;
        b_more            = true;

        for (burst = 0U; b_more && (burst < MQTT_PUBLISH_BURST); burst++)
        {
            if (!g_b_pending)
            {
                g_b_pending = comms_link_next(&g_pending);
            }

            b_more = g_b_pending && publish_pending();
        }
    }
}


static void
initialize (void)
{
    g_b_initialized  = true;
    g_status.port    = (uint16_t) UTK_MQTT_BROKER_PORT;
    g_b_broker_valid = (0 != ipaddr_aton(UTK_MQTT_BROKER_ADDRESS,
                                         &g_broker));

    if (g_b_broker_valid)
    {
        g_status.broker[0] = ip4_addr1(ip_2_ip4(&g_broker));
        g_status.broker[1] = ip4_addr2(ip_2_ip4(&g_broker));
        g_status.broker[2] = ip4_addr3(ip_2_ip4(&g_broker));
        g_status.broker[3] = ip4_addr4(ip_2_ip4(&g_broker));
    }
    else
    {
        g_status.last_error = (int32_t) ERR_ARG;
    }

    enter_state(LWIP_UTK_MQTT_IDLE);
}


/* One step of the session state machine (see the diagram at the top). */
static void
poll_state (uint32_t now_ms, bool b_net_ready)
{
    switch ((lwip_utk_mqtt_state_t) g_status.state)
    {
        case LWIP_UTK_MQTT_IDLE:
            if (b_net_ready)
            {
                try_connect();
            }
            break;

        case LWIP_UTK_MQTT_CONNECTING:
            /* Success or refusal arrives through connection_cb().  Give up
               here if WiFi dropped or the broker never answers: closing
               with mqtt_disconnect() raises no callback of its own. */
            if (!b_net_ready
                || ((now_ms - g_state_since_ms) >= MQTT_CONNECT_TIMEOUT_MS))
            {
                mqtt_disconnect(&g_client);
                start_backoff(b_net_ready ? (int32_t) ERR_TIMEOUT
                                          : (int32_t) ERR_CONN);
            }
            break;

        case LWIP_UTK_MQTT_CONNECTED:
            if (!b_net_ready)
            {
                /* WiFi dropped: close cleanly so the next link starts a
                   fresh session instead of waiting out keep-alive. */
                mqtt_disconnect(&g_client);
                g_status.disconnects++;
                start_backoff((int32_t) ERR_CONN);
            }
            else if (0U == mqtt_client_is_connected(&g_client))
            {
                g_status.disconnects++;
                start_backoff((int32_t) ERR_CLSD);
            }
            else
            {
                service_connected(now_ms);
            }
            break;

        case LWIP_UTK_MQTT_BACKOFF:
            if (b_net_ready && !g_b_net_was_ready)
            {
                /* The network has just come back: the old back-off was
                   measuring an outage that is over, so start afresh. */
                g_status.retry_delay_ms = 0U;
                try_connect();
            }
            else if (b_net_ready
                     && ((now_ms - g_state_since_ms)
                         >= g_status.retry_delay_ms))
            {
                try_connect();
            }
            else
            {
                /* Keep waiting. */
            }
            break;

        default:
            enter_state(LWIP_UTK_MQTT_IDLE);
            break;
    }
}


void
lwip_utk_mqtt_poll (struct netif * p_netif)
{
    uint32_t now_ms;
    bool     b_net_ready;

    if (!g_b_initialized)
    {
        initialize();
    }

    if (g_b_broker_valid && (NULL != p_netif))
    {
        now_ms      = sys_now();
        b_net_ready = netif_is_up(p_netif) && netif_is_link_up(p_netif)
                   && !ip4_addr_isany_val(*netif_ip4_addr(p_netif));
        poll_state(now_ms, b_net_ready);
        g_b_net_was_ready = b_net_ready;
    }
}


void
lwip_utk_mqtt_get_status (lwip_utk_mqtt_status_t * p_status)
{
    if (NULL != p_status)
    {
        *p_status = g_status;
    }
}
