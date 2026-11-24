/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * comms_link.h
 * Boundary between the comms API (application side) and the MQTT session
 * in the radio task.  Only the radio task calls these functions.
 *
 * Plain C types only: this header is included by lwIP/pico-sdk code whose
 * size_t clashes with the T-Kernel headers.
 */
#ifndef COMMS_LINK_H
#define COMMS_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define COMMS_LINK_TOPIC_MAX     32U
#define COMMS_LINK_PAYLOAD_MAX   224U

typedef struct
{
    char     topic[COMMS_LINK_TOPIC_MAX];
    char     payload[COMMS_LINK_PAYLOAD_MAX];
    uint16_t payload_len;
    uint8_t  qos;
    uint8_t  b_retain;
} comms_link_msg_t;

/* Next message to publish, highest priority first; false if none due. */
bool comms_link_next(comms_link_msg_t * p_msg);

/* Session up or down.  On (re)connect every snapshot is resent. */
void comms_link_set_connected(bool b_connected);

/* A complete payload received on robot/command. */
void comms_link_on_command(uint8_t const * p_data, uint32_t len);

#endif /* COMMS_LINK_H */
