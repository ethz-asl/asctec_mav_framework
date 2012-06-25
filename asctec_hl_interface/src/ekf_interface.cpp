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

#include "ekf_interface.h"
#include <tf/tf.h>
#include "helper.h"

EKFInterface::EKFInterface(ros::NodeHandle & nh, CommPtr & comm) :
  nh_(nh), pnh_("~"), comm_(comm)
{
  state_pub_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("ekf_state_out", 1);
  state_sub_ = nh.subscribe("ekf_state_in", 1, &EKFInterface::stateCallback, this);

  comm_->registerCallback(HLI_PACKET_ID_EKF_STATE, &EKFInterface::processEkfData, this);
}

void EKFInterface::processEkfData(uint8_t * buf, uint32_t bufLength)
{
  static uint32_t seq = 0;
  if (state_pub_.getNumSubscribers() > 0)
  {
    HLI_EKF_STATE * state = (HLI_EKF_STATE*)buf;
    sensor_fusion_comm::ExtEkfPtr msg(new sensor_fusion_comm::ExtEkf);

    msg->header.stamp = ros::Time(state->timestamp * 1.0e-6);
    msg->header.seq = seq;

    msg->linear_acceleration.x = helper::asctecAccToSI(state->acc_x);
    msg->linear_acceleration.y = helper::asctecAccToSI(state->acc_y);
    msg->linear_acceleration.z = helper::asctecAccToSI(state->acc_z);

    msg->angular_velocity.x = helper::asctecOmegaToSI(state->ang_vel_roll);
    msg->angular_velocity.y = helper::asctecOmegaToSI(state->ang_vel_pitch);
    msg->angular_velocity.z = helper::asctecOmegaToSI(state->ang_vel_yaw);

    msg->flag = sensor_fusion_comm::ExtEkf::current_state;

    msg->state.resize(HLI_EKF_STATE_SIZE);

    for (uint32_t i = 0; i < HLI_EKF_STATE_SIZE; i++)
    {
      msg->state[i] = state->state[i];
    }

    state_pub_.publish(msg);
  }

  seq++;
}

void EKFInterface::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
{
  ekf_state_msg_.timestamp = static_cast<uint64_t> (msg->header.stamp.toSec() * 1.0e6);

  switch (msg->flag)
  {
    case sensor_fusion_comm::ExtEkf::current_state:
      ekf_state_msg_.flag = HLI_EKF_STATE_CURRENT_STATE;
      break;
    case sensor_fusion_comm::ExtEkf::initialization:
      ekf_state_msg_.flag = HLI_EKF_STATE_INITIALIZATION;
      break;
    case sensor_fusion_comm::ExtEkf::state_correction:
      ekf_state_msg_.flag = HLI_EKF_STATE_STATE_CORRECTION;
      break;
    default:
      ekf_state_msg_.flag = HLI_EKF_STATE_INITIALIZATION;
  }

  if (msg->state.size() != HLI_EKF_STATE_SIZE)
  {
    ROS_WARN_STREAM("size of incoming state ("<< msg->state.size() <<") != size of state in the HL processor ("<< HLI_EKF_STATE_SIZE <<"), not sending!");
    return;
  }

  // we don't need acc and omega on the HL, so skip this here

  for (unsigned int i = 0; i < HLI_EKF_STATE_SIZE; i++)
  {
    ekf_state_msg_.state[i] = msg->state[i];
  }

  comm_->sendPacket(HLI_PACKET_ID_EKF_STATE, ekf_state_msg_);
}

