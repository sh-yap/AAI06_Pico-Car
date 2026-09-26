/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * comms_demo.h
 * Fake-data driver for testing the comms API (make ... COMMS_DEMO=1).
 */
#ifndef COMMS_DEMO_H
#define COMMS_DEMO_H

#include <stdbool.h>

#ifndef COMMS_DEMO
#define COMMS_DEMO 0
#endif

#if COMMS_DEMO
bool comms_demo_start(void);
#endif

#endif /* COMMS_DEMO_H */
