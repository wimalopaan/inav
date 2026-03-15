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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/crc.h"

#include "io/serial.h"
#include "io/servo_sbus.h"
#include "rx/sbus_channels.h"

#if defined(USE_SERVO_SBUS)

#define SERVO_SBUS_UART_BAUD            100000
#define SERVO_SBUS_OPTIONS              (SBUS_PORT_OPTIONS | SERIAL_INVERTED | SERIAL_UNIDIR)

#define MAX_SBUS_PORT_COUNT 2

// all sbus instances have their own fram buffer.
// This is not strictly neccessary in the first place (all should output the same data)
// But maybe in the future ... ;-)

typedef struct sbusPort_s {
  serialPort_t *port; 
  sbusFrame_t sbusFrame;
} sbusPort_t;

static sbusPort_t sbusPorts[MAX_SBUS_PORT_COUNT];

static void sbusServoSetFrameValue(uint8_t const index, uint16_t const value, sbusFrame_t* const frame)
{
      if (!frame) {
        return;
      }
      switch(index) {
          case 0: frame->channels.chan0 = sbusEncodeChannelValue(value); break;
          case 1: frame->channels.chan1 = sbusEncodeChannelValue(value); break;
          case 2: frame->channels.chan2 = sbusEncodeChannelValue(value); break;
          case 3: frame->channels.chan3 = sbusEncodeChannelValue(value); break;
          case 4: frame->channels.chan4 = sbusEncodeChannelValue(value); break;
          case 5: frame->channels.chan5 = sbusEncodeChannelValue(value); break;
          case 6: frame->channels.chan6 = sbusEncodeChannelValue(value); break;
          case 7: frame->channels.chan7 = sbusEncodeChannelValue(value); break;
          case 8: frame->channels.chan8 = sbusEncodeChannelValue(value); break;
          case 9: frame->channels.chan9 = sbusEncodeChannelValue(value); break;
          case 10: frame->channels.chan10 = sbusEncodeChannelValue(value); break;
          case 11: frame->channels.chan11 = sbusEncodeChannelValue(value); break;
          case 12: frame->channels.chan12 = sbusEncodeChannelValue(value); break;
          case 13: frame->channels.chan13 = sbusEncodeChannelValue(value); break;
          case 14: frame->channels.chan14 = sbusEncodeChannelValue(value); break;
          case 15: frame->channels.chan15 = sbusEncodeChannelValue(value); break;
          case 16: frame->channels.flags = value > PWM_RANGE_MIDDLE ? (frame->channels.flags | SBUS_FLAG_CHANNEL_DG1) : (frame->channels.flags & ~SBUS_FLAG_CHANNEL_DG1) ; break;
          case 17: frame->channels.flags = value > PWM_RANGE_MIDDLE ? (frame->channels.flags | SBUS_FLAG_CHANNEL_DG2) : (frame->channels.flags & ~SBUS_FLAG_CHANNEL_DG2) ; break;
          default:
              break;
      }
}

static void resetSbusPort(sbusPort_t * const sbusPortToReset, serialPort_t * const serialPort)
{
    if (!sbusPortToReset) {
      return;
    }
    memset(sbusPortToReset, 0, sizeof(sbusPort_t));
    sbusPortToReset->port = serialPort;
}

static void clearSbusFrame(sbusFrame_t* const frame) {
    if (!frame) {
      return;
    }
    frame->syncByte = 0x0F;
    frame->channels.flags = 0;
    for(uint8_t i = 0; i < 16; ++i) {
      sbusServoSetFrameValue(i, 1500, frame);
    }
    frame->endByte = 0x00;
}

static void sbusServoAllocatePorts(void)
{
    uint8_t portIndex = 0;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SERVO_SERIAL);
    while (portConfig && portIndex < MAX_SBUS_PORT_COUNT) {
        sbusPort_t * const sbusPort = &sbusPorts[portIndex];
        if (sbusPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t * const serialPort = openSerialPort(portConfig->identifier, FUNCTION_SERVO_SERIAL, NULL, NULL, SERVO_SBUS_UART_BAUD, MODE_TX, SERVO_SBUS_OPTIONS);
        if (serialPort) {
            resetSbusPort(sbusPort, serialPort);
            clearSbusFrame(&sbusPort->sbusFrame);
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_SERVO_SERIAL);
    }
}

bool sbusServoInitialize(void)
{
    memset(sbusPorts, 0, sizeof(sbusPorts));
    sbusServoAllocatePorts();
    return true;
}

void sbusServoUpdate(uint8_t const index, uint16_t const value)
{
    for(sbusPort_t* sbusPort = &sbusPorts[0]; (sbusPort && (sbusPort != &sbusPorts[MAX_SBUS_PORT_COUNT])); ++sbusPort) {
      sbusServoSetFrameValue(index, value, &sbusPort->sbusFrame);
    }
}

void sbusServoSendUpdate(void)
{
    for(const sbusPort_t* sbusPort = &sbusPorts[0]; (sbusPort && (sbusPort != &sbusPorts[MAX_SBUS_PORT_COUNT])); ++sbusPort)
    {
      if (!sbusPort->port) {
        continue;
      }
      if (!isSerialTransmitBufferEmpty(sbusPort->port)) {
          continue;
      }
      else {
        const uint8_t * const data =  (const uint8_t *)&sbusPort->sbusFrame;
        if (data) {
          serialWriteBuf(sbusPort->port, data, sizeof(sbusFrame_t));
        }
      }
    }
}

#endif
