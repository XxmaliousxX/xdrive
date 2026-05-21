/**
 * \file api.h
 *
 * PROS API header provides high-level user functionality
 *
 * Contains declarations for use by typical VEX programmers using PROS.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * \copyright Copyright (c) 2017-2024, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_API_H_
#define _PROS_API_H_

#ifdef __cplusplus
#include <cerrno>
#include <cmath>
#include <cstddef> // IWYU pragma: keep
#include <cstdint> // IWYU pragma: keep
#include <cstdio>
#include <cstdlib>
#include <iostream> // IWYU pragma: keep
#else /* (not) __cplusplus */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif /* __cplusplus */

#include "pros/adi.h" // IWYU pragma: keep
#include "pros/ai_vision.h" // IWYU pragma: keep
#include "pros/colors.h" // IWYU pragma: keep
#include "pros/device.h" // IWYU pragma: keep
#include "pros/distance.h" // IWYU pragma: keep
#include "pros/error.h" // IWYU pragma: keep
#include "pros/ext_adi.h" // IWYU pragma: keep
#include "pros/gps.h" // IWYU pragma: keep
#include "pros/imu.h" // IWYU pragma: keep
#include "pros/link.h" // IWYU pragma: keep
#include "pros/llemu.h" // IWYU pragma: keep
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/rotation.h" // IWYU pragma: keep
#include "pros/rtos.h" // IWYU pragma: keep
#include "pros/screen.h" // IWYU pragma: keep
#include "pros/vision.h" // IWYU pragma: keep

#ifdef __cplusplus
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/ai_vision.hpp" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/gps.hpp" // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/link.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp" // IWYU pragma: keep
#include "pros/misc.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/optical.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp" // IWYU pragma: keep
#include "pros/screen.hpp" // IWYU pragma: keep
#include "pros/vision.hpp" // IWYU pragma: keep
#endif

#endif  // _PROS_API_H_
