/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Local MQTT broker settings for the WIFI_MQTT=1 profile.
 *
 * Copy this file to config/mqtt_config.h, set the broker address, then
 * change UTK_MQTT_CONFIG_SET from 0 to 1.  The local copy is ignored by Git
 * because the broker address changes from network to network.
 */
#ifndef UTK3_MQTT_CONFIG_H
#define UTK3_MQTT_CONFIG_H

#define UTK_MQTT_CONFIG_SET   0

#if !UTK_MQTT_CONFIG_SET
#error "config/mqtt_config.h: placeholder values. Set the broker address, then set UTK_MQTT_CONFIG_SET to 1."
#endif

/* Dotted IPv4 address of the machine running the broker (e.g. Mosquitto). */
#define UTK_MQTT_BROKER_ADDRESS   "192.168.1.100"
#define UTK_MQTT_BROKER_PORT      1883U

/* Must be unique per broker: a second client with the same id evicts us. */
#define UTK_MQTT_CLIENT_ID        "aai06-pico-car"

#endif /* UTK3_MQTT_CONFIG_H */
