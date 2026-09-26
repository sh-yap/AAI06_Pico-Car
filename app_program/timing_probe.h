/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * timing_probe.h
 * Control-loop wake-up latency probe (COMMS_DEMO=1 builds only).
 */
#ifndef TIMING_PROBE_H
#define TIMING_PROBE_H

#include <stdbool.h>

#include "comms_demo.h"

#if COMMS_DEMO
bool timing_probe_start(void);
#endif

#endif /* TIMING_PROBE_H */
