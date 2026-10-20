/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* lwIP 2.2.x configuration for the micro T-Kernel Pico W owner task. */
#ifndef UTK3_LWIPOPTS_H
#define UTK3_LWIPOPTS_H

#include <stdint.h>

/* Phase 2 has one execution context: the processor-1 radio-owner task. */
#define NO_SYS                          1
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ARP                        1
#define LWIP_ICMP                       (TM_WIFI_STATIC || TM_WIFI_DHCP)
#define LWIP_RAW                        (TM_WIFI_STATIC || TM_WIFI_DHCP)
#define LWIP_UDP                        TM_WIFI_DHCP
#define LWIP_TCP                        (TM_WIFI_TCP || TM_WIFI_MQTT)
/* The altcp shim is only needed by application protocols written against it
 * Nothing in this tree uses it; turn it on here if you add something that
 * does. */
#define LWIP_ALTCP                      0
#define LWIP_ALTCP_TLS                  0
#define LWIP_DHCP                       TM_WIFI_DHCP
/* No ARP address-conflict probe on the offered lease.  After a quick
 * power-cycle the phone hotspot still answers ARP for the Pico's previous
 * association, the probe reports a conflict, and DHCP declines and retries
 * every 10 s indefinitely (seen on hardware: link up, no address after a
 * replug).  The hotspot hands out the lease, so the probe adds nothing.
 * Raspberry Pi's own Pico W lwIP examples disable it too. */
#define LWIP_DHCP_DOES_ACD_CHECK        0
#define LWIP_AUTOIP                     0
#define LWIP_DNS                        TM_WIFI_DHCP
#define LWIP_IGMP                       0
#define ARP_QUEUEING                    (TM_WIFI_STATIC || TM_WIFI_DHCP)

#define LWIP_TIMERS                     1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_EXT_STATUS_CALLBACK  0

#define MEM_ALIGNMENT                   4
#if TM_WIFI_TCP || TM_WIFI_MQTT
#define MEM_SIZE                        (12 * 1024)
#else
#define MEM_SIZE                        (8 * 1024)
#endif
#define MEMP_NUM_PBUF                   16
#if TM_WIFI_MQTT
/* Outgoing MQTT data waiting for TCP: room for a burst of telemetry and
 * events (largest payload ~220 bytes).  QoS 1 messages each hold a request
 * slot until the broker acknowledges them. */
#define MQTT_OUTPUT_RINGBUF_SIZE        1024
#define MQTT_REQ_MAX_IN_FLIGHT          8
#endif
#if TM_WIFI_MQTT
/* lwIP's own timers plus the MQTT client's cyclic keep-alive timer. */
#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 2)
#else
#define MEMP_NUM_SYS_TIMEOUT            8
#endif
#define PBUF_POOL_SIZE                  8
#define PBUF_POOL_BUFSIZE               1600

/* Phase 7 opens exactly one outbound connection and never listens.  The
 * send window stays small on purpose: the gate is stop-and-wait, so a large
 * window would only cost RAM the kernel qualification also needs. */
#if TM_WIFI_TCP || TM_WIFI_MQTT
/* Each finished connection lingers in TIME_WAIT for 2MSL while still holding
 * a pool entry, and the cumulative image now opens two connections in
 * sequence (Phase 7, then Phase 8).  Two entries would force lwIP to
 * start reaping to make room. */
#define MEMP_NUM_TCP_PCB                6
#define MEMP_NUM_TCP_PCB_LISTEN         1
#define MEMP_NUM_TCP_SEG                16
#define TCP_MSS                         1460
#if TM_WIFI_MQTT
/* Give up on an unreachable broker after ~9 s (3+6 s of SYN back-off)
 * instead of the default ~20 s, so the MQTT retry loop gets another go
 * sooner once the network comes back. */
#define TCP_SYNMAXRTX                   2
#endif
#define TCP_SND_BUF                     (2 * TCP_MSS)
#define TCP_WND                         (2 * TCP_MSS)
#define TCP_LISTEN_BACKLOG              0
#endif

#define LWIP_STATS                      0
#define LWIP_DEBUG                      0
#define LWIP_PLATFORM_DIAG(x)           ((void)0)
void lwip_utk_assert(const char *message);
#define LWIP_PLATFORM_ASSERT(x)         lwip_utk_assert(x)
uint32_t lwip_utk_rand(void);
#define LWIP_RAND()                     lwip_utk_rand()

#endif
