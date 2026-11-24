/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * build_info.c
 * Build identification.  The makefile recompiles this file on every build
 * (see pico_rp2040.mk), so the stamp always matches the flashed image.
 */

#include "build_info.h"

char const g_build_id[] = __DATE__ " " __TIME__;
