/*

 Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef __DEKF_H__
#define __DEKF_H__

#include <math.h>
#include <HL_interface.h>
#include <ekf/rtwtypes.h>
#include "uart.h"

#define DEKF_CORRECTION_SMOOTING_LENGTH 100
#define DEKF_WATCHDOG_TIMEOUT 10  // timeout in [s]

typedef struct
{
  real32_T current_state[HLI_EKF_STATE_SIZE];
  real32_T last_state[HLI_EKF_STATE_SIZE];
  real32_T acc[3];
  real32_T ang_vel[3];
  real32_T dt;
  real32_T q_tmp[4];
  HLI_EKF_STATE state_out;
  HLI_EKF_STATE state_in;
  PacketInfo * packet_info;
  uint64_t last_time;
  HLI_EXT_POSITION * pos_ctrl_input;
  real32_T ctrl_correction[6];
  real32_T ctrl_correction_step[6];
  int ctrl_correction_count;
  int propagate_state;
  unsigned int watchdog;
  char initialize_event;
} DekfContext;

void DEKF_init(DekfContext * self, HLI_EXT_POSITION * pos_ctrl_input);

void DEKF_sendState(DekfContext * self, int64_t  timestamp);

void DEKF_step(DekfContext * self, int64_t timestamp);

char DEKF_getInitializeEvent(DekfContext * self);

inline void initState(DekfContext * self);

inline void correctState(DekfContext * self);

inline void writeControllerOutput(DekfContext * self);

inline real32_T yawFromQuaternion(const real32_T q[4]);
inline void quaternionMultiplication(const real32_T q1[4], const real32_T q2[4], real32_T q[4]);

/// convert float to int with correct rounding
inline int float2Int(float x);

/// convert float to short with correct rounding
inline short float2Short(float x);



#endif
