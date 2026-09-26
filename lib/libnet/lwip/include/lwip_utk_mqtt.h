/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * lwip_utk_mqtt.h
 * MQTT client session owned by the radio service task.
 *
 * Every function here calls into lwIP, so it must only be called from the
 * radio-owner task (NO_SYS=1: lwIP has exactly one execution context).
 * Other tasks read the published snapshot through cyw43_utk_get_status().
 */
#ifndef LWIP_UTK_MQTT_H
#define LWIP_UTK_MQTT_H

#include <stdint.h>

/* Longest command payload kept for the application; longer ones are cut. */
#define LWIP_UTK_MQTT_CMD_MAX   64U

typedef enum
{
    LWIP_UTK_MQTT_IDLE = 0,     /* waiting for WiFi + IP address        */
    LWIP_UTK_MQTT_CONNECTING,   /* TCP/MQTT handshake in progress       */
    LWIP_UTK_MQTT_CONNECTED,    /* session up, heartbeat running        */
    LWIP_UTK_MQTT_BACKOFF       /* lost or refused; waiting to retry    */
} lwip_utk_mqtt_state_t;

typedef struct
{
    uint32_t state;             /* lwip_utk_mqtt_state_t                */
    int32_t  last_error;        /* last lwIP err_t or connection status */
    uint32_t connect_attempts;
    uint32_t connects;          /* successful sessions since boot       */
    uint32_t disconnects;
    uint32_t retry_delay_ms;    /* current back-off step                */
    uint32_t published;
    uint32_t publish_errors;
    uint32_t cmd_count;         /* commands received since boot         */
    uint32_t cmd_len;
    uint8_t  cmd_payload[LWIP_UTK_MQTT_CMD_MAX];
    uint8_t  broker[4];
    uint16_t port;
} lwip_utk_mqtt_status_t;

struct netif;

void lwip_utk_mqtt_poll(struct netif *p_netif);
void lwip_utk_mqtt_get_status(lwip_utk_mqtt_status_t *p_status);

#endif /* LWIP_UTK_MQTT_H */
