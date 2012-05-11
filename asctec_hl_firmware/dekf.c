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

#include "dekf.h"
#include "LL_HL_comm.h"

#include <ekf/autogen_ekf_propagation.h>
#include <ekf/autogen_ekf_propagation_initialize.h>

// conversion from AscTec acceleration values to m/s^2
const real32_T DEKF_ASCTEC_ACC_TO_SI = 9.81e-3;

// conversion from AscTec turn rates to rad/s
const real32_T DEKF_ASCTEC_OMEGA_TO_SI = 0.015 * M_PI / 180.0;

void DEKF_init(DekfContext * self, HLI_EXT_POSITION * pos_ctrl_input){
  int i;

  // init state vector
  for(i=0; i<HLI_EKF_STATE_SIZE; i++){
    self->last_state[i] = 0;
    self->current_state[i] = 0;
  }
  self->last_state[6] = 1;     // unit quaternion
  self->current_state[6] = 1;  // unit quaternion

  self->last_time = 0;
  self->ctrl_correction_count = 0;
  self->propagate_state = FALSE;
  self->watchdog = 0;

  autogen_ekf_propagation_initialize();

  self->pos_ctrl_input = pos_ctrl_input;

  self->packet_info = registerPacket(HLI_PACKET_ID_EKF_STATE, &self->state_in);
}

void DEKF_sendState(DekfContext * self, int64_t timestamp)
{
  int i = 0;
  self->state_out.timestamp = timestamp;

  // TODO: smoothing of data to make Nyquist happy ;-)
  // acceleration, angular velocities following the ENU convention (x front, y left, z up)
  self->state_out.acc_x = LL_1khz_attitude_data.acc_x;
  self->state_out.acc_y = -LL_1khz_attitude_data.acc_y;
  self->state_out.acc_z = -LL_1khz_attitude_data.acc_z;
  self->state_out.ang_vel_roll = LL_1khz_attitude_data.angvel_roll;
  self->state_out.ang_vel_pitch = -LL_1khz_attitude_data.angvel_pitch;
  self->state_out.ang_vel_yaw = -LL_1khz_attitude_data.angvel_yaw;

  for (i = 0; i < HLI_EKF_STATE_SIZE; i++)
  {
    self->state_out.state[i] = self->current_state[i];
  }

  writePacket2Ringbuffer(HLI_PACKET_ID_EKF_STATE, (unsigned char*)&self->state_out, sizeof(self->state_out));
}

void DEKF_step(DekfContext * self, int64_t timestamp)
{
  self->watchdog ++;

  int i = 0;

  int64_t idt = timestamp - self->last_time;
  self->dt = (real32_T)idt * 1.0e-6;
  self->last_time = timestamp;

  // if idt > 100ms or negative, something went really wrong
  if(idt > 100000 || idt < 0)
    return;

  if(self->watchdog > 1000 * DEKF_WATCHDOG_TIMEOUT){
    self->watchdog = 1000 * DEKF_WATCHDOG_TIMEOUT;
    self->propagate_state = FALSE;
  }

  // bring data to SI units and ENU coordinates
  self->acc[0] = ((real32_T)LL_1khz_attitude_data.acc_x) * DEKF_ASCTEC_ACC_TO_SI;
  self->acc[1] = -((real32_T)LL_1khz_attitude_data.acc_y) * DEKF_ASCTEC_ACC_TO_SI;
  self->acc[2] = -((real32_T)LL_1khz_attitude_data.acc_z) * DEKF_ASCTEC_ACC_TO_SI;

  self->ang_vel[0] = ((real32_T)LL_1khz_attitude_data.angvel_roll) * DEKF_ASCTEC_OMEGA_TO_SI;
  self->ang_vel[1] = -((real32_T)LL_1khz_attitude_data.angvel_pitch) * DEKF_ASCTEC_OMEGA_TO_SI;
  self->ang_vel[2] = -((real32_T)LL_1khz_attitude_data.angvel_yaw) * DEKF_ASCTEC_OMEGA_TO_SI;

//  // disable system inputs for testing
//  self->acc[0] = 0;
//  self->acc[1] = 0;
//  self->acc[2] = 9.81;
//  self->ang_vel[0] = 0;
//  self->ang_vel[1] = 0;
//  self->ang_vel[2] = 0;

  if (self->propagate_state == TRUE)
  {
    autogen_ekf_propagation(self->last_state, self->acc, self->ang_vel, self->dt, self->current_state);
  }
  else
  {
    // hold state, set velocity to zero
    self->current_state[3] = 0;
    self->current_state[4] = 0;
    self->current_state[5] = 0;

    //TODO: yaw ?!?
  }


  self->initialize_event = 0;

  if (self->packet_info->updated)
  {
    self->packet_info->updated = 0;
    self->propagate_state = TRUE;
    self->watchdog = 0;

    if (self->state_in.flag == HLI_EKF_STATE_STATE_CORRECTION)
    {
      // correct states
      correctState(self);
    }
    else if (self->state_in.flag == HLI_EKF_STATE_INITIALIZATION)
    {
      initState(self);
      self->ctrl_correction_count = 0;
      self->initialize_event = 1;
      writeControllerOutput(self);
    }
  }

  writeControllerOutput(self);

  for (i = 0; i < HLI_EKF_STATE_SIZE; i++)
  {
    self->last_state[i] = self->current_state[i];
  }

}

void initState(DekfContext * self)
{
  for (int i = 0; i < HLI_EKF_STATE_SIZE; i++)
  {
    self->current_state[i] = self->state_in.state[i];
  }
}

void correctState(DekfContext * self)
{
  // position
  self->current_state[0] += self->state_in.state[0];
  self->current_state[1] += self->state_in.state[1];
  self->current_state[2] += self->state_in.state[2];

  // velocity
  self->current_state[3] += self->state_in.state[3];
  self->current_state[4] += self->state_in.state[4];
  self->current_state[5] += self->state_in.state[5];

  // orientation
  self->q_tmp[0] = self->current_state[6];
  self->q_tmp[1] = self->current_state[7];
  self->q_tmp[2] = self->current_state[8];
  self->q_tmp[3] = self->current_state[9];

  quaternionMultiplication(self->q_tmp, &self->state_in.state[6], &self->current_state[6]);

  // gyro bias
  self->current_state[10] += self->state_in.state[10];
  self->current_state[11] += self->state_in.state[11];
  self->current_state[12] += self->state_in.state[12];

  // acceleration bias
  self->current_state[13] += self->state_in.state[13];
  self->current_state[14] += self->state_in.state[14];
  self->current_state[15] += self->state_in.state[15];

  self->ctrl_correction_count = DEKF_CORRECTION_SMOOTING_LENGTH;

  // pos_ctrl is NED !!!
  self->ctrl_correction[0] = self->current_state[0] - ((real32_T) self->pos_ctrl_input->x) * 1e-3;
  self->ctrl_correction[1] = self->current_state[1] - (-(real32_T) self->pos_ctrl_input->y) * 1e-3;
  self->ctrl_correction[2] = self->current_state[2] - (-(real32_T) self->pos_ctrl_input->z) * 1e-3;
  self->ctrl_correction[3] = self->current_state[3] - ((real32_T) self->pos_ctrl_input->vX) * 1e-3;
  self->ctrl_correction[4] = self->current_state[4] - (-(real32_T) self->pos_ctrl_input->vY) * 1e-3;
  self->ctrl_correction[5] = self->current_state[5] - (-(real32_T) self->pos_ctrl_input->vZ) * 1e-3;

  for(int i=0; i<6; i++){
    self->ctrl_correction_step[i] = self->ctrl_correction[i] / (real32_T)DEKF_CORRECTION_SMOOTING_LENGTH;
  }
//  self->last_correction[i] = self->current_state[i] -

}

void writeControllerOutput(DekfContext * self)
{
  if (self->ctrl_correction_count > 0)
  {
    for (int i = 0; i < 6; i++)
    {
      self->ctrl_correction[i] -= self->ctrl_correction_step[i];
    }
    self->ctrl_correction_count--;
  }
  else if (self->ctrl_correction_count == 0)
//  else
  {
    for (int i = 0; i < 6; i++)
    {
      self->ctrl_correction[i] = 0;
    }
    self->ctrl_correction_count--;
  }

  self->pos_ctrl_input->bitfield = EXT_POSITION_BYPASS_FILTER;
  self->pos_ctrl_input->x = float2Int((self->current_state[0]-self->ctrl_correction[0]) * 1000.0);
  self->pos_ctrl_input->y = -float2Int((self->current_state[1]-self->ctrl_correction[1]) * 1000.0);
  self->pos_ctrl_input->z = -float2Int((self->current_state[2]-self->ctrl_correction[2]) * 1000.0);
  self->pos_ctrl_input->vX = float2Short((self->current_state[3]-self->ctrl_correction[3]) * 1000.0);
  self->pos_ctrl_input->vY = -float2Short((self->current_state[4]-self->ctrl_correction[4]) * 1000.0);
  self->pos_ctrl_input->vZ = -float2Short((self->current_state[5]-self->ctrl_correction[5]) * 1000.0);

  real32_T * const q = &self->current_state[6];

  real32_T yaw = yawFromQuaternion(q);
  self->pos_ctrl_input->heading = 360000 - (int)(((yaw < 0 ? yaw + 2 * M_PI : yaw) * 180.0 / M_PI) * 1000.0);

  //    statusData.debug1 = pos_ctrl_input->heading;
  //    statusData.debug2 = yaw*180.0/M_PI*1000;

//      pos_ctrl_input->count = sdkLoops;
  self->pos_ctrl_input->qualX = 100;
  self->pos_ctrl_input->qualY = 100;
  self->pos_ctrl_input->qualZ = 100;
  self->pos_ctrl_input->qualVx = 100;
  self->pos_ctrl_input->qualVy = 100;
  self->pos_ctrl_input->qualVz = 100;
}

char DEKF_getInitializeEvent(DekfContext * self)
{
  return self->initialize_event;
}

real32_T yawFromQuaternion(const real32_T q[4])
{
  real32_T x = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
  real32_T y = 2.0 * (q[3] * q[0] + q[1] * q[2]);
  real32_T yaw = 0;
  real32_T r = 0;

  //  yaw = atan2(y, x);

  //alternate atan2: http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
  real32_T abs_y = fabs(y) + 1e-10; // kludge to prevent 0/0 condition
  if (x >= 0)
  {
    r = (x - abs_y) / (x + abs_y);
    yaw = r * (0.1963 * r * r - 0.9817) + M_PI / 4.0;
  }
  else
  {
    r = (x + abs_y) / (abs_y - x);
    yaw = r * (0.1963 * r * r - 0.9817) + 3.0 * M_PI / 4.0;
  }
  if (y < 0)
    yaw = -yaw; // negate if in quad III or IV;

  return yaw;
}

void quaternionMultiplication(const real32_T q1[4], const real32_T q2[4], real32_T q[4])
{
  q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  q[2] = q1[0] * q2[2] + q1[2] * q2[0] - q1[1] * q2[3] + q1[3] * q2[1];
  q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

int float2Int(float x)
{
  // TODO: range checking?
  return x >= 0 ? (int)(x + 0.5) : (int)(x - 0.5);
}

short float2Short(float x)
{
  // TODO: range checking?
  return x >= 0 ? (short)(x + 0.5) : (short)(x - 0.5);
}
