/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <common/time.h>

#ifdef USE_SERVO_VESC

typedef struct {
    bool halfDuplex;
} vescMasterConfig_t;

PG_DECLARE(vescMasterConfig_t, vescMasterConfig);

bool vescInitialize(void);
void vescSet(uint8_t index, uint16_t value);
void vescMasterHandle(timeUs_t); // @100Hz

uint16_t vescVoltage(void);
uint16_t vescCurrentIn(void);
uint16_t vescCurrent(void);
uint16_t vescRpm(uint8_t index);
uint16_t vescTemp(uint8_t index);

#endif
