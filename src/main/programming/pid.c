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

#include "platform.h"

#ifdef USE_PROGRAMMING_FRAMEWORK

#include "common/utils.h"
#include "build/debug.h"
#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "navigation/navigation_private.h"

#include "programming/pid.h"
#include "programming/logic_condition.h"
#include "programming/global_variables.h"

#include "flight/imu.h"
#include "flight/mixer.h"

#include "rx/rx.h"

#include "stdlib.h"

EXTENDED_FASTRAM programmingPidState_t programmingPidState[MAX_PROGRAMMING_PID_COUNT];
static bool pidsInitiated = false;

PG_REGISTER_ARRAY_WITH_RESET_FN(programmingPid_t, MAX_PROGRAMMING_PID_COUNT, programmingPids, PG_PROGRAMMING_PID, 2);

void pgResetFn_programmingPids(programmingPid_t *instance)
{
    for (int i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
        RESET_CONFIG(programmingPid_t, &instance[i],
            .enabled = 0,
            .setpoint = {
                .type = LOGIC_CONDITION_OPERAND_TYPE_VALUE,
                .value = 0
            },
            .measurement = {
                .type = LOGIC_CONDITION_OPERAND_TYPE_VALUE,
                .value = 0
            },
            .gains = {
                .P = 0,
                .I = 0,
                .D = 0,
                .FF = 0,
            }
        );
    }
}

// todo:
// eigene PID Controller für balance-bot
// mit der hohen update Frequenz
// LQR

// void programmingPidUpdateTask_old(timeUs_t currentTimeUs)
// {
//     static bool active = false;
//     static timeUs_t previousUpdateTimeUs;
//     const float dT = US2S(currentTimeUs - previousUpdateTimeUs);
    
//     if (!pidsInitiated) {
//         programmingPidInit();
//         pidsInitiated = true;
//     }

//     const int16_t thr = (mixerThrottleCommand - 1500) / 15; // throttle 
    
//     debug[2] = mixerThrottleCommand;
    
//     for (uint8_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
//         if (programmingPids(i)->enabled) {
//             int setpoint = logicConditionGetOperandValue(programmingPids(i)->setpoint.type, programmingPids(i)->setpoint.value);
//             setpoint += thr;
            
//             const int measurement = logicConditionGetOperandValue(programmingPids(i)->measurement.type, programmingPids(i)->measurement.value);

//             if (active) {
//                 if (abs(measurement) > 300) { // 30deg
//                     active = false;
//                 }
//             }
//             else {
//                 if (abs(measurement) < 30) { // 3deg
//                     active = true;
//                 }                
//             }
            
//             if (!active) { // 30deg
//                 programmingPidState[i].output = 0;
//                 programmingPidState[i].outfilter.state = 0;
//             }
//             else {
//                 const float mf = pt1FilterApply(&programmingPidState[i].pitchfilter, measurement);
//                 float m1 = mf;
//                 float expo = (mf * mf) / (1000 - programmingPids(i)->gains.FF);
//                 if (mf >= 0) {
//                     m1 += expo;
//                 }
//                 else {
//                     m1 -= expo;                
//                 }
                
//                 float v1 = navPidApply2(
//                     &programmingPidState[i].controller,
//                     setpoint,
//                     m1,
//                     dT,
//                     -1000,
//                     1000,
//                     PID_LIMIT_INTEGRATOR
//                 );
                
//                 programmingPidState[i].output = v1;
//                 pt1FilterApply(&programmingPidState[i].outfilter, v1);
                
//             }
            
//         }
//     }

//     previousUpdateTimeUs = currentTimeUs;
// }

// void programmingPidInit_old(void)
// {
//     for (uint8_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
//         navPidInit(
//             &programmingPidState[i].controller,
//             programmingPids(i)->gains.P / 1000.0f,
//             programmingPids(i)->gains.I / 1000.0f,
//             programmingPids(i)->gains.D / 1000.0f,
//             0.0f,
//             // programmingPids(i)->gains.FF / 1000.0f,
//             5.0f,
//             0.0f
//         );
//         programmingPidState[i].outfilter.alpha = 0.25f;
//         programmingPidState[i].outfilter.state = 0.0f;
//         programmingPidState[i].pitchfilter.alpha = 0.001f;
//         programmingPidState[i].pitchfilter.state = 0.0f;
//     }
// }

void programmingPidUpdateTask(timeUs_t currentTimeUs)
{
    static bool active[MAX_PROGRAMMING_PID_COUNT] = {false, true, true, true};
    static timeUs_t previousUpdateTimeUs;
    const float dT = US2S(currentTimeUs - previousUpdateTimeUs);
    
    static int32_t errorLPF[MAX_PROGRAMMING_PID_COUNT] = {50, 50, 50, 50}; // 1/10 Hz 
    static int32_t dtermLPF[MAX_PROGRAMMING_PID_COUNT] = {200, 200, 200, 200};
    
    if (!pidsInitiated) {
        programmingPidInit();
        pidsInitiated = true;
    }

    // const int16_t thr = (mixerThrottleCommand - 1500) / 8; // throttle 
    
    for (uint8_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
        if (programmingPids(i)->enabled) {
 
            const float gain = gvGet(3 * i + 0) / 10.0f;

            const int32_t newErrorLPF = gvGet(3 * i + 1);
            if (errorLPF[i] != newErrorLPF && newErrorLPF > 0) {
                errorLPF[i] = newErrorLPF;
                programmingPidState[i].controller.error_filter_state.RC = 0;
                programmingPidState[i].controller.errorLpfHz = newErrorLPF / 10.0f;
            }
            const int32_t newDtermLPF = gvGet(3 * i + 2);
            if (dtermLPF[i] != newDtermLPF && newDtermLPF > 0) {
                dtermLPF[i] = newDtermLPF;
                programmingPidState[i].controller.dterm_filter_state.RC = 0;
                programmingPidState[i].controller.dTermLpfHz = newDtermLPF / 10.0f;
            }
            
            int setpoint = logicConditionGetOperandValue(programmingPids(i)->setpoint.type, programmingPids(i)->setpoint.value);

            // setpoint += thr;
            
            const int measurement = logicConditionGetOperandValue(programmingPids(i)->measurement.type, programmingPids(i)->measurement.value);

            if (i == 0) { // special treatment
                if (active[i]) {
                    if (abs(measurement) > 300) { // 30deg
                        active[i] = false;
                    }
                }
                else {
                    if (abs(measurement) < 30) { // 3deg
                        active[i] = true;
                    }                
                }
            }
            
            if (!active[i]) { // 30deg
                programmingPidState[i].output = 0;
            }
            else {
                float max = 2100.0f;
                if (i == 1) {
                    max = gvGet(6); // 1/10 degree
                }
                
                programmingPidState[i].output  = navPidApply3(
                    &programmingPidState[i].controller,
                    setpoint,
                    measurement,
                    dT,
                    -max,
                    max,
                    0, // flags
                    gain,
                    1.0f // dTerm Scaler (additional to gain)
                );
                // debug[0] = setpoint;
                // debug[1] = programmingPidState[i].controller.errorLpfHz;
                // debug[2] = programmingPidState[i].controller.proportional;
                // debug[3] = programmingPidState[i].controller.integrator;
                // debug[4] = programmingPidState[i].controller.derivative;
                // debug[5] = programmingPidState[i].controller.feedForward;
                // debug[6] = programmingPidState[i].controller.output_constrained;
                // debug[7] = thr;
            }
        }
    }
    previousUpdateTimeUs = currentTimeUs;
}

void programmingPidInit(void)
{
    for (uint8_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
        navPidInit(
            &programmingPidState[i].controller,
            programmingPids(i)->gains.P / 1000.0f,
            programmingPids(i)->gains.I / 1000.0f,
            programmingPids(i)->gains.D / 1000.0f,
            programmingPids(i)->gains.FF / 1000.0f,
            20.0f, // dTerm LPF
            5.0f  // error LPF
        );
    }
}

int32_t programmingPidGetOutput(uint8_t i) {
    return programmingPidState[constrain(i, 0, MAX_PROGRAMMING_PID_COUNT)].output;
}

void programmingPidReset(void)
{
    for (uint8_t i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
        navPidReset(&programmingPidState[i].controller);
    }
}

#endif
