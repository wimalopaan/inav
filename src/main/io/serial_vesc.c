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

#if defined(USE_SERVO_VESC)

#define VESC_UART_BAUD                  115200
#define VESC_UART_OPTIONS              (SERIAL_UNIDIR)

#define MAX_VESC_PORT_COUNT 2

typedef enum CommPacketId {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_HANDBRAKE,
    COMM_SET_DETECT,
    COMM_SET_SERVO_POS,
    COMM_SET_MCCONF,
    COMM_GET_MCCONF,
    COMM_GET_MCCONF_DEFAULT,
    COMM_SET_APPCONF,
    COMM_GET_APPCONF,
    COMM_GET_APPCONF_DEFAULT,
    COMM_SAMPLE_PRINT,
    COMM_TERMINAL_CMD,
    COMM_PRINT,
    COMM_ROTOR_POSITION,
    COMM_EXPERIMENT_SAMPLE,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_MOTOR_R_L,
    COMM_DETECT_MOTOR_FLUX_LINKAGE,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL_FOC,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_DECODED_PPM,
    COMM_GET_DECODED_ADC,
    COMM_GET_DECODED_CHUK,
    COMM_FORWARD_CAN,
    COMM_SET_CHUCK_DATA,
    COMM_CUSTOM_APP_DATA,
    COMM_NRF_START_PAIRING,
    COMM_GPD_SET_FSW,
    COMM_GPD_BUFFER_NOTIFY,
    COMM_GPD_BUFFER_SIZE_LEFT,
    COMM_GPD_FILL_BUFFER,
    COMM_GPD_OUTPUT_SAMPLE,
    COMM_GPD_SET_MODE,
    COMM_GPD_FILL_BUFFER_INT8,
    COMM_GPD_FILL_BUFFER_INT16,
    COMM_GPD_SET_BUFFER_INT_SCALE,
    COMM_GET_VALUES_SETUP,
    COMM_SET_MCCONF_TEMP,
    COMM_SET_MCCONF_TEMP_SETUP,
    COMM_GET_VALUES_SELECTIVE,
    COMM_GET_VALUES_SETUP_SELECTIVE,
    COMM_EXT_NRF_PRESENT,
    COMM_EXT_NRF_ESB_SET_CH_ADDR,
    COMM_EXT_NRF_ESB_SEND_DATA,
    COMM_EXT_NRF_ESB_RX_DATA,
    COMM_EXT_NRF_SET_ENABLED,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
    COMM_DETECT_APPLY_ALL_FOC,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
    COMM_ERASE_NEW_APP_ALL_CAN,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN,
    COMM_PING_CAN,
    COMM_APP_DISABLE_OUTPUT,
    COMM_TERMINAL_CMD_SYNC,
    COMM_GET_IMU_DATA,
    COMM_BM_CONNECT,
    COMM_BM_ERASE_FLASH_ALL,
    COMM_BM_WRITE_FLASH,
    COMM_BM_REBOOT,
    COMM_BM_DISCONNECT,
    COMM_BM_MAP_PINS_DEFAULT,
    COMM_BM_MAP_PINS_NRF5X,
    COMM_ERASE_BOOTLOADER,
    COMM_ERASE_BOOTLOADER_ALL_CAN,
    COMM_PLOT_INIT,
    COMM_PLOT_DATA,
    COMM_PLOT_ADD_GRAPH,
    COMM_PLOT_SET_GRAPH,
    COMM_GET_DECODED_BALANCE,
    COMM_BM_MEM_READ,
    COMM_WRITE_NEW_APP_DATA_LZO,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
    COMM_BM_WRITE_FLASH_LZO,
    COMM_SET_CURRENT_REL,
    COMM_CAN_FWD_FRAME,
    COMM_SET_BATTERY_CUT,
    COMM_SET_BLE_NAME,
    COMM_SET_BLE_PIN,
    COMM_SET_CAN_MODE,
    COMM_GET_IMU_CALIBRATION,
    COMM_GET_MCCONF_TEMP,
    
    // Custom configuration for hardware
    COMM_GET_CUSTOM_CONFIG_XML,
    COMM_GET_CUSTOM_CONFIG,
    COMM_GET_CUSTOM_CONFIG_DEFAULT,
    COMM_SET_CUSTOM_CONFIG,
    
    // BMS commands
    COMM_BMS_GET_VALUES,
    COMM_BMS_SET_CHARGE_ALLOWED,
    COMM_BMS_SET_BALANCE_OVERRIDE,
    COMM_BMS_RESET_COUNTERS,
    COMM_BMS_FORCE_BALANCE,
    COMM_BMS_ZERO_CURRENT_OFFSET,
    
    // FW updates commands for different HW types
    COMM_JUMP_TO_BOOTLOADER_HW,
    COMM_ERASE_NEW_APP_HW,
    COMM_WRITE_NEW_APP_DATA_HW,
    COMM_ERASE_BOOTLOADER_HW,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW,
    COMM_ERASE_NEW_APP_ALL_CAN_HW,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW,
    COMM_ERASE_BOOTLOADER_ALL_CAN_HW,
    
    COMM_SET_ODOMETER,
    
    // Power switch commands
    COMM_PSW_GET_STATUS,
    COMM_PSW_SWITCH,
    
    COMM_BMS_FWD_CAN_RX,
    COMM_BMS_HW_DATA,
    COMM_GET_BATTERY_CUT,
    COMM_BM_HALT_REQ,
    COMM_GET_QML_UI_HW,
    COMM_GET_QML_UI_APP,
    COMM_CUSTOM_HW_DATA,
    COMM_QMLUI_ERASE,
    COMM_QMLUI_WRITE,
    
    // IO Board
    COMM_IO_BOARD_GET_ALL,
    COMM_IO_BOARD_SET_PWM,
    COMM_IO_BOARD_SET_DIGITAL,
    
    COMM_BM_MEM_WRITE,
    COMM_BMS_BLNC_SELFTEST,
    COMM_GET_EXT_HUM_TMP,
    COMM_GET_STATS,
    COMM_RESET_STATS,
    
    // Lisp
    COMM_LISP_READ_CODE,
    COMM_LISP_WRITE_CODE,
    COMM_LISP_ERASE_CODE,
    COMM_LISP_SET_RUNNING,
    COMM_LISP_GET_STATS,
    COMM_LISP_PRINT,
    
    COMM_BMS_SET_BATT_TYPE,
    COMM_BMS_GET_BATT_TYPE,
    
    COMM_LISP_REPL_CMD,
} CommPacketId_t;


typedef enum vescState_e {SendDuty = 0, 
                          RequestValues} vescState_t;

typedef struct vescFrameDuty_s {
    uint8_t start;
    uint8_t len;
    uint8_t cmd;
    uint8_t duty[4];
    uint8_t crc[2];
    uint8_t end;
} vescFrameDuty_t;

typedef struct vescFrameGetValues_s {
    uint8_t start;
    uint8_t len;
    uint8_t cmd;
    uint8_t crc[2];
    uint8_t end;
} vescFrameValues_t;

typedef struct vescFrame_s {
    vescFrameDuty_t duty;
    vescFrameValues_t getValues;        
} vescFrame_t;

typedef enum vescProcessState_e {
    Process_WaitForREsponse = 0,
    Process_Length,
    Process_Type,
    Process_Data,
    Process_CrcH,
    Process_CrcL,
} vescProcessState_t;

typedef struct vescProcess_s {
    vescProcessState_t state;
    uint16_t csum;
    uint16_t length;
    uint16_t counter;
    uint8_t  type;
    uint8_t  data[256];
    uint16_t crc;
    uint16_t packages;
    uint16_t bytes;
} vescProcess_t;

typedef struct vescInfo_s {
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t FWTestVersionNumber;
    uint8_t HWType;
    char    name[16];
} vescInfo_t;

typedef struct vescValues_s {
    uint16_t temperature;
    uint16_t temperatureMotor;
    int32_t current;
    int32_t currentIn;
    int32_t rpm;
    uint16_t voltage;
    uint32_t consumption;
    uint8_t fault;
} vescValues_t;

typedef struct vescPort_s {
    serialPort_t *port;
    vescFrame_t frames;
    vescState_t state;
    int32_t duty;
    vescProcess_t process;
    vescInfo_t info;
    vescValues_t values;
} vescPort_t;

static vescPort_t vescPorts[MAX_VESC_PORT_COUNT];

static void fillVescFrames(vescFrame_t* const frames){
    frames->duty.start = 0x02;
    frames->duty.len = 0x05;
    frames->duty.cmd = COMM_SET_DUTY;
    frames->duty.end = 0x03;
    
    frames->getValues.start = 0x02;
    frames->getValues.len = 0x01;
    frames->getValues.cmd = COMM_GET_VALUES;
    uint16_t crc = crc16_ccitt(0, frames->getValues.cmd);
    frames->getValues.crc[0] = crc >> 8;
    frames->getValues.crc[1] = crc >> 0;
    frames->getValues.end = 0x03;
}

static void resetVescPort(vescPort_t * const vescPortToReset, serialPort_t * const serialPort) {
    if (!vescPortToReset) {
        return;
    }
    memset(vescPortToReset, 0, sizeof(vescPort_t));
    vescPortToReset->port = serialPort;
}

static void clearVescFrame(vescFrame_t* const frame) {
    if (!frame) {
        return;
    }
    memset(frame, 0, sizeof(vescFrame_t));
}

static void vescAllocatePorts(void) {
    uint8_t portIndex = 0;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VESC);
    while (portConfig && portIndex < MAX_VESC_PORT_COUNT) {
        vescPort_t * const vescPort = &vescPorts[portIndex];
        if (vescPort->port) {
            portIndex++;
            continue;
        }
        
        serialPort_t * const serialPort = openSerialPort(portConfig->identifier, FUNCTION_VESC, NULL, NULL, VESC_UART_BAUD, MODE_RXTX, VESC_UART_OPTIONS);
        if (serialPort) {
            resetVescPort(vescPort, serialPort);
            clearVescFrame(&vescPort->frames);
            fillVescFrames(&vescPort->frames);
            portIndex++;
        }
        
        portConfig = findNextSerialPortConfig(FUNCTION_VESC);
    }
}

bool vescInitialize(void)
{
    memset(vescPorts, 0, sizeof(vescPorts));
    vescAllocatePorts();
    return true;
}

void vescSet(uint8_t const index, uint16_t const value) {
    if (index < MAX_VESC_PORT_COUNT) {
        vescPort_t* const vescPort = &vescPorts[index];
        vescPort->duty = ((value - 1500) * (int32_t)100000) / 500;
    }
}

static void sendDuty(vescPort_t* const port) {
    port->frames.duty.duty[0] = port->duty >> 24;    
    port->frames.duty.duty[1] = port->duty >> 16;    
    port->frames.duty.duty[2] = port->duty >> 8;    
    port->frames.duty.duty[3] = port->duty >> 0;    
    uint16_t crc = crc16_ccitt(0, port->frames.duty.cmd);
    for(uint8_t i = 0; i < 4; ++i) {
        crc = crc16_ccitt(crc, port->frames.duty.duty[i]);
    }
    port->frames.duty.crc[0] = crc >> 8;
    port->frames.duty.crc[1] = crc >> 0;
    
    serialWriteBuf(port->port, (const uint8_t*)&port->frames.duty, sizeof(vescFrameDuty_t));
}

static void sendRequest(vescPort_t* const port) {
    serialWriteBuf(port->port, (const uint8_t*)&port->frames.getValues, sizeof(vescFrameValues_t));    
}

static void vescFsm(vescPort_t* const port) {
    if (!port->port) {
        return;
    }
    if (!isSerialTransmitBufferEmpty(port->port)) {
        return;
    }
    else {
        
        switch(port->state) {
        case SendDuty:
            sendDuty(port);
            port->state = RequestValues;
            break;
        case RequestValues:
            sendRequest(port);
            port->state = SendDuty;
            break;
        }
    }
}

static void decode(vescPort_t* const port) {
    const uint8_t type = port->process.type; 
    const uint8_t* const data = &port->process.data[0];
    
    if (type == COMM_FW_VERSION) {
        uint16_t k = 0;
        port->info.versionMajor = (uint8_t)data[k++];
        port->info.versionMinor = (uint8_t)data[k++];
        for(uint16_t i = 0; i < sizeof(port->info.name); ++i) {
            port->info.name[i] = (char)data[k++];
            if (port->info.name[i] == '\0')
                break;
        }
        k += 12; // UUID
        k++; // pairing
        port->info.FWTestVersionNumber = (uint8_t)data[k++];
        port->info.HWType = (uint8_t)data[k++]; // enum?
    }
    else if (type == COMM_GET_VALUES) {
        port->values.temperature = ((int32_t)data[0]) << 8;
        port->values.temperature |= ((int32_t)data[1]);
        
        port->values.temperatureMotor = ((int32_t)data[2]) << 8;
        port->values.temperatureMotor |= ((int32_t)data[3]);
        
        port->values.current = ((int32_t)data[4]) << 24;
        port->values.current |= ((int32_t)data[5]) << 16;
        port->values.current |= ((int32_t)data[6]) << 8;
        port->values.current |= ((int32_t)data[7]);
        
        if (port->values.current < 0) {
            port->values.current = -port->values.current;
        }
        
        port->values.currentIn = ((int32_t)data[8]) << 24;
        port->values.currentIn |= ((int32_t)data[9]) << 16;
        port->values.currentIn |= ((int32_t)data[10]) << 8;
        port->values.currentIn |= ((int32_t)data[11]);
        
        if (port->values.currentIn < 0) {
            port->values.currentIn = -port->values.currentIn;
        }
        
        port->values.rpm = ((int32_t)data[22]) << 24;
        port->values.rpm |= ((int32_t)data[23]) << 16;
        port->values.rpm |= ((int32_t)data[24]) << 8;
        port->values.rpm |= ((int32_t)data[25]);
        
        if (port->values.rpm < 0) {
            port->values.rpm = -port->values.rpm;
        }
        
        port->values.voltage = ((int32_t)data[26]) << 8;
        port->values.voltage |= ((int32_t)data[27]);
        
        port->values.consumption = ((int32_t)data[28]) << 24;
        port->values.consumption |= ((int32_t)data[29]) << 16;
        port->values.consumption |= ((int32_t)data[30]) << 8;
        port->values.consumption |= ((int32_t)data[31]);
        
        port->values.fault = (uint8_t)data[52];    
    }
}    

static void vescProcess(vescPort_t* const port, const uint8_t b) {
    port->process.bytes++;
    switch (port->process.state) {
    case Process_WaitForREsponse:
        if (b == 0x02) {
            port->process.state = Process_Length;
        }
        else {
            port->process.state = Process_WaitForREsponse;
        }
        break;
    case Process_Length:
        port->process.length = b;
        port->process.csum = 0;
        port->process.state = Process_Type;
        break;
    case Process_Type:
        --port->process.length;
        port->process.csum = crc16_ccitt(port->process.csum, b);
        port->process.type = b;
        port->process.counter = 0;
        port->process.state = Process_Data;
        break;
    case Process_Data:
        port->process.csum = crc16_ccitt(port->process.csum, b);
        port->process.data[port->process.counter++] = b;
        if (port->process.counter == port->process.length) {
            port->process.state = Process_CrcH;
        }
        break;
    case Process_CrcH:
        port->process.crc = b << 8;
        port->process.state = Process_CrcL;
        break;
    case Process_CrcL:
        port->process.crc |= b;
        if (port->process.csum == port->process.crc) {
            decode(port);
            ++port->process.packages;
        }
        port->process.state = Process_WaitForREsponse;
        break;
    }
}

static void vescReadReply(vescPort_t* const port) {
    if (!port->port) {
        return;
    }
    while (serialRxBytesWaiting(port->port)) {
        const uint8_t c = serialRead(port->port);
        vescProcess(port, c);
    }
}

void vescMasterHandle(timeUs_t) {   
    for(vescPort_t* vescPort = &vescPorts[0]; (vescPort != &vescPorts[MAX_VESC_PORT_COUNT]); ++vescPort) {
        vescReadReply(vescPort);
        vescFsm(vescPort);
    }
}
uint16_t vescVoltage(void) {
    for(vescPort_t* vescPort = &vescPorts[0]; (vescPort != &vescPorts[MAX_VESC_PORT_COUNT]); ++vescPort) {
        if (vescPort->port) {
            return vescPort->values.voltage;
        }        
    }
    return 0;
}
uint16_t vescCurrentIn(void) {
    for(vescPort_t* vescPort = &vescPorts[0]; (vescPort != &vescPorts[MAX_VESC_PORT_COUNT]); ++vescPort) {
        if (vescPort->port) {
            return vescPort->values.currentIn;
        }        
    }
    return 0;
}
uint16_t vescCurrent(void) {
    for(vescPort_t* vescPort = &vescPorts[0]; (vescPort != &vescPorts[MAX_VESC_PORT_COUNT]); ++vescPort) {
        if (vescPort->port) {
            return vescPort->values.current;
        }        
    }
    return 0;
}
uint16_t vescRpm(uint8_t const index) {
    if (index < MAX_VESC_PORT_COUNT) {
        const vescPort_t* const vescPort = &vescPorts[index];
        if (vescPort->port) {
            return vescPort->values.rpm;
        }        
    }
    return 0;
}
uint16_t vescTemp(uint8_t const index) {
    if (index < MAX_VESC_PORT_COUNT) {
        const vescPort_t* const vescPort = &vescPorts[index];
        if (vescPort->port) {
            return vescPort->values.temperature;
        }        
    }
    return 0;
}
#endif
