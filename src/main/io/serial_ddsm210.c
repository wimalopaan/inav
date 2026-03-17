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

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/crc.h"

#include "io/serial.h"

#if defined(USE_DDSM_MOTOR)

#define DDSM_UART_BAUD                  115200
#define DDSM_UART_OPTIONS              (SERIAL_UNIDIR)

#define MAX_DDSM_PORT_COUNT 2

typedef struct ddsmFrame_s {
    uint8_t id;
    uint8_t cmd;
    uint8_t data[7];
    uint8_t crc8;
} ddsmFrame_t;

typedef struct ddsmPort_s {
    serialPort_t *port;
    ddsmFrame_t frame;
} ddsmPort_t;

static ddsmPort_t ddsmPorts[MAX_DDSM_PORT_COUNT];

static void resetDdsmPort(ddsmPort_t * const ddsmPortToReset, serialPort_t * const serialPort)
{
    if (!ddsmPortToReset) {
        return;
    }
    memset(ddsmPortToReset, 0, sizeof(ddsmPort_t));
    ddsmPortToReset->port = serialPort;
}

static void clearDdsmFrame(ddsmFrame_t* const frame) {
    if (!frame) {
        return;
    }
    memset(frame, 0, sizeof(ddsmFrame_t));
}

static void ddsmAllocatePorts(void)
{
    uint8_t portIndex = 0;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_DDSM_SERIAL);
    while (portConfig && portIndex < MAX_DDSM_PORT_COUNT) {
        ddsmPort_t * const ddsmPort = &ddsmPorts[portIndex];
        if (ddsmPort->port) {
            portIndex++;
            continue;
        }
        
        serialPort_t * const serialPort = openSerialPort(portConfig->identifier, FUNCTION_DDSM_SERIAL, NULL, NULL, DDSM_UART_BAUD, MODE_TX, DDSM_UART_OPTIONS);
        if (serialPort) {
            resetDdsmPort(ddsmPort, serialPort);
            clearDdsmFrame(&ddsmPort->frame);
            portIndex++;
        }
        
        portConfig = findNextSerialPortConfig(FUNCTION_DDSM_SERIAL);
    }
}

bool ddsmInitialize(void)
{
    memset(ddsmPorts, 0, sizeof(ddsmPorts));
    ddsmAllocatePorts();
    return true;
}

static void frame_crc(ddsmFrame_t* const frame) {
    uint8_t crc = 0;
    crc = crc8_maxim(crc, frame->id);
    crc = crc8_maxim(crc, frame->cmd);
    for(uint8_t i = 0; i < 7; ++i) {
        crc = crc8_maxim(crc, frame->data[i]);    
    }
    frame->crc8 = crc;
}

static void fillDdsmVelocityFrame(ddsmFrame_t* const frame, int16_t const velocity)
{
    frame->id = 0x01;
    frame->cmd = 0x64;
    frame->data[0] = velocity >> 8;
    frame->data[1] = velocity;
    frame_crc(frame);
}

void ddsmSet(uint8_t const index, uint16_t const value)
{
    if ((index == 0) && (ddsmPorts[0].port)) {
        const int16_t velocity = value - 1500;
        fillDdsmVelocityFrame(&ddsmPorts[0].frame, velocity);      
    }
}

void ddsmUpdate(void)
{
    for(const ddsmPort_t* ddsmPort = &ddsmPorts[0]; (ddsmPort != &ddsmPorts[MAX_DDSM_PORT_COUNT]); ++ddsmPort)
    {
        if (!ddsmPort->port) {
            continue;
        }
        if (!isSerialTransmitBufferEmpty(ddsmPort->port)) {
            continue;
        }
        else {
            const uint8_t * const data =  (const uint8_t *)&ddsmPort->frame;
            if (data) {
                serialWriteBuf(ddsmPort->port, data, sizeof(ddsmFrame_t));
            }
        }
    }
}

#endif
