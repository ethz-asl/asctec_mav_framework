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

#include "ssdk_interface.h"
#include "helper.h"
#include <asctec_hl_comm/mav_ctrl.h>

SSDKInterface::SSDKInterface(ros::NodeHandle & nh, CommPtr & comm):
  nh_(nh),
  pnh_("~/ssdk"),
  comm_(comm),
  have_config_(false)
{
  ros::NodeHandle _pnh("~");
  _pnh.param("frame_id", frame_id_, std::string("fcu"));

  ROS_WARN_COND(!pnh_.hasParam("omega_0_xy"), "no ssdk parameters available, position control on the HLP will not work!");

  pose_sub_ = nh_.subscribe("pose", 1, &SSDKInterface::cbPose, this);
  state_sub_ = nh_.subscribe("state", 1, &SSDKInterface::cbState, this);
  odometry_sub_ = nh_.subscribe("odometry", 1, &SSDKInterface::cbOdometry, this);
  debug_pub_ = nh_.advertise<asctec_hl_comm::DoubleArrayStamped> ("debug", 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("current_pose", 1);

  config_server_ = new SSDKConfigServer(pnh_);
  SSDKConfigServer::CallbackType f = boost::bind(&SSDKInterface::cbSSDKConfig, this, _1, _2);
  config_server_->setCallback(f);

  comm_->registerCallback(HLI_PACKET_ID_SSDK_DEBUG, &SSDKInterface::processDebugData, this);
  comm_->registerCallback(HLI_PACKET_ID_SSDK_STATUS, &SSDKInterface::processStatusData, this);
}

SSDKInterface::~SSDKInterface()
{
  delete config_server_;
}


void SSDKInterface::tfCallback()
{
  tf::StampedTransform pose;
  static tf::StampedTransform last_pose;
  ros::Time tf_time(0);
  if (tf_listener_.canTransform(config_.tf_ref_frame_id, config_.tf_tracked_frame_id, tf_time))
  {
    tf_listener_.lookupTransform(config_.tf_ref_frame_id, config_.tf_tracked_frame_id, tf_time, pose);

    // only send new poses to HLP checking one coordinate should be sufficient since it will never be the same
    if (pose.getOrigin() != last_pose.getOrigin())
    {
      const tf::Vector3 & pos = pose.getOrigin();
      const tf::Quaternion & q = pose.getRotation();
      sendPoseToAP(pos.x(), pos.y(), pos.z(), tf::getYaw(q), 100);
      last_pose = pose;
    }
  }
}

void SSDKInterface::cbPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  const geometry_msgs::Point & pos = msg->pose.pose.position;
  double yaw = tf::getYaw(msg->pose.pose.orientation);

  char qual = 100;

  for (unsigned int i = 0; i < msg->pose.covariance.size(); i++)
  {
    if (msg->pose.covariance[i] > 1)
    {
      qual = 0;
    }
  }

  sendPoseToAP(pos.x, pos.y, pos.z, yaw, qual);
}


void SSDKInterface::sendPoseToAP(const double & x, const double & y, const double & z, const double & yaw,
                               const unsigned char & qual)
{
  static HLI_EXT_POSITION pose;
  static uint8_t count = 0;

  pose.x = (int)(x * 1000);
  pose.y = (int)(y * 1000);
  pose.z = (int)(z * 1000);
  pose.heading = helper::yaw2asctec(yaw);
  pose.qualX = qual;
  pose.qualY = qual;
  pose.qualZ = qual;

  pose.vX = 0;
  pose.vY = 0;
  pose.vZ = 0;
  pose.qualVx = 0;
  pose.qualVy = 0;
  pose.qualVz = 0;

  pose.bitfield = 0;

  pose.count = count++;

  comm_->sendPacket(HLI_PACKET_ID_POSITION_UPDATE, pose);
}


void SSDKInterface::cbSSDKConfig(asctec_hl_interface::SSDKConfig & config, uint32_t level)
{
  config.p31 = config.omega_0_xy * config.omega_0_xy;
  config.p32 = config.omega_0_z * config.omega_0_z;
  config.p33 = config.omega_0_xy * config.zeta_xy * 2.0;
  config.p34 = config.omega_0_z * config.zeta_z * 2.0;

  if (level & asctec_hl_interface::SSDK_SEND_PARAMETERS)
  {
    if(!sendParameters(config)){
      // TODO: not sure if this is maybe too harsh, but we would be in a somewhat undefined state
      ROS_FATAL("Could not send SSDK parameters to the HLP! Shutting down the node");
      ros::shutdown();
    }
    config.send = false;
  }

  if (level & asctec_hl_interface::SSDK_TF_CHANGED)
  {
    if(tf_callback_.connected())
    {
      tf_listener_.removeTransformsChangedListener(tf_callback_);
    }
  }

  config_ = config;
  have_config_ = true;

  if ((level & asctec_hl_interface::SSDK_TF_CHANGED) && config.listen_on_tf)
  {
    if(!tf_callback_.connected())
      tf_callback_ = tf_listener_.addTransformsChangedListener(boost::bind(&SSDKInterface::tfCallback, this));
  }
}


void SSDKInterface::cbState(const sensor_fusion_comm::ExtStatePtr & msg)
{
  // TODO: untested, use that with care !!!

  // receive state (position, yaw, velocity) and directly pass it to the HL position controller by bypassing the
  // state observer on the HL

  HLI_EXT_POSITION state;

  double yaw = tf::getYaw(msg->pose.orientation);

  char qual = 100;

  state.x = static_cast<int> (msg->pose.position.x * 1000);
  state.y = static_cast<int> (msg->pose.position.y * 1000);
  state.z = static_cast<int> (msg->pose.position.z * 1000);
  state.heading = static_cast<int> (helper::yaw2asctec(yaw));
  state.qualX = qual;
  state.qualY = qual;
  state.qualZ = qual;

  state.vX = static_cast<short> (msg->velocity.x * 1000);
  state.vY = static_cast<short> (msg->velocity.y * 1000);
  state.vZ = static_cast<short> (msg->velocity.z * 1000);
  state.qualVx = qual;
  state.qualVy = qual;
  state.qualVz = qual;

  state.bitfield = EXT_POSITION_BYPASS_FILTER;

  comm_->sendPacket(HLI_PACKET_ID_POSITION_UPDATE, state);
}

void SSDKInterface::cbOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  // TODO: untested, use that with care !!!

  // receive state (position, yaw, velocity) and directly pass it to the HL position controller by bypassing the
  // state observer on the HL

  if(msg->child_frame_id != msg->header.frame_id){
    ROS_WARN_STREAM_THROTTLE(1, "header.frame_id and child_frame_id don't match, not sending to HLP. got "<<msg->child_frame_id << " / "<<msg->header.frame_id);
    return;
  }

  HLI_EXT_POSITION state;

  double yaw = tf::getYaw(msg->pose.pose.orientation);

  char qual = 100;

  state.x = static_cast<int> (msg->pose.pose.position.x * 1000);
  state.y = static_cast<int> (msg->pose.pose.position.y * 1000);
  state.z = static_cast<int> (msg->pose.pose.position.z * 1000);
  state.heading = static_cast<int> (helper::yaw2asctec(yaw));
  state.qualX = qual;
  state.qualY = qual;
  state.qualZ = qual;

  // TODO: rotate or not?
  geometry_msgs::Vector3 v = msg->twist.twist.linear;

  state.vX = static_cast<short> (v.x * 1000);
  state.vY = static_cast<short> (v.y * 1000);
  state.vZ = static_cast<short> (v.z * 1000);
  state.qualVx = qual;
  state.qualVy = qual;
  state.qualVz = qual;

  state.bitfield = EXT_POSITION_BYPASS_FILTER;

  comm_->sendPacket(HLI_PACKET_ID_POSITION_UPDATE, state);
}

void SSDKInterface::processStatusData(uint8_t * buf, uint32_t length)
{
  HLI_SSDK_STATUS *status = (HLI_SSDK_STATUS*)buf;
  if(status->have_parameters == 0 && have_config_)
    sendParameters(config_);
}


void SSDKInterface::processDebugData(uint8_t * buf, uint32_t length)
{
  static unsigned int seq = 0;
  HLI_SSDK_DEBUG *data = (HLI_SSDK_DEBUG*)buf;
  int size = sizeof(HLI_SSDK_DEBUG) / sizeof(short);

  if (pose_pub_.getNumSubscribers() > 0)
  {
    // TODO: currently limited to +- 65m, move this to somewhere else
    // this is still NED, so convert to ENU
    geometry_msgs::PoseStampedPtr msg(new geometry_msgs::PoseStamped);

    msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
    msg->header.frame_id = frame_id_;
    msg->header.seq = seq;

    msg->pose.position.x = helper::debug2Double(data->debug14);
    msg->pose.position.y = -helper::debug2Double(data->debug15);
    msg->pose.position.z = -helper::debug2Double(data->debug16);

    geometry_msgs::Quaternion & q = msg->pose.orientation;
    helper::angle2quaternion(helper::debug2Double(data->debug29), -helper::debug2Double(data->debug30),
                             -helper::debug2Double(data->debug31), &q.w, &q.x, &q.y, &q.z);
    pose_pub_.publish(msg);
  }

  if (debug_pub_.getNumSubscribers() > 0)
  {
    asctec_hl_comm::DoubleArrayStampedPtr msg(new asctec_hl_comm::DoubleArrayStamped);

    msg->data.resize(size);

    msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
    msg->header.frame_id = frame_id_;
    msg->header.seq = seq;

    msg->data[0] = helper::debug2Double(data->debug01);
    msg->data[1] = helper::debug2Double(data->debug02);
    msg->data[2] = helper::debug2Double(data->debug03);
    msg->data[3] = helper::debug2Double(data->debug04);
    msg->data[4] = helper::debug2Double(data->debug05);
    msg->data[5] = helper::debug2Double(data->debug06);
    msg->data[6] = helper::debug2Double(data->debug07);
    msg->data[7] = helper::debug2Double(data->debug08);
    msg->data[8] = helper::debug2Double(data->debug09);
    msg->data[9] = helper::debug2Double(data->debug10);
    msg->data[10] = helper::debug2Double(data->debug11);
    msg->data[11] = helper::debug2Double(data->debug12);
    msg->data[12] = helper::debug2Double(data->debug13);
    msg->data[13] = helper::debug2Double(data->debug14);
    msg->data[14] = helper::debug2Double(data->debug15);
    msg->data[15] = helper::debug2Double(data->debug16);
    msg->data[16] = helper::debug2Double(data->debug17);
    msg->data[17] = helper::debug2Double(data->debug18);
    msg->data[18] = helper::debug2Double(data->debug19);
    msg->data[19] = helper::debug2Double(data->debug20);
    msg->data[20] = helper::debug2Double(data->debug21);
    msg->data[21] = helper::debug2Double(data->debug22);
    msg->data[22] = helper::debug2Double(data->debug23);
    msg->data[23] = helper::debug2Double(data->debug24);
    msg->data[24] = helper::debug2Double(data->debug25);
    msg->data[25] = helper::debug2Double(data->debug26);
    msg->data[26] = helper::debug2Double(data->debug27);
    msg->data[27] = helper::debug2Double(data->debug28);
    msg->data[28] = helper::debug2Double(data->debug29);
    msg->data[29] = helper::debug2Double(data->debug30);
    msg->data[30] = helper::debug2Double(data->debug31);
    msg->data[31] = helper::debug2Double(data->debug32);
    msg->data[32] = helper::debug2Double(data->debug33);
    msg->data[33] = helper::debug2Double(data->debug34);
    msg->data[34] = helper::debug2Double(data->debug35);
    msg->data[35] = helper::debug2Double(data->debug36);
    msg->data[36] = helper::debug2Double(data->debug37);
    msg->data[37] = helper::debug2Double(data->debug38);
    msg->data[38] = helper::debug2Double(data->debug39);

    debug_pub_.publish(msg);
  }
  seq++;
}

bool SSDKInterface::sendParameters(const asctec_hl_interface::SSDKConfig & config)
{
  HLI_SSDK_PARAMS params;
  params.p01 = helper::param2Fixpoint(config.p1);
  params.p02 = helper::param2Fixpoint(config.p2);
  params.p03 = helper::param2Fixpoint(config.p3);
  params.p04 = helper::param2Fixpoint(config.p4);
  params.p05 = helper::param2Fixpoint(config.p5);
  params.p06 = helper::param2Fixpoint(config.p6);
  params.p07 = helper::param2Fixpoint(config.p7);
  params.p08 = helper::param2Fixpoint(config.p8);
  params.p09 = helper::param2Fixpoint(config.p9);
  params.p10 = helper::param2Fixpoint(config.p10);
  params.p11 = helper::param2Fixpoint(config.p11);
  params.p12 = helper::param2Fixpoint(config.p12);
  params.p13 = helper::param2Fixpoint(config.p13);
  params.p14 = helper::param2Fixpoint(config.p14);
  params.p15 = helper::param2Fixpoint(config.p15);
  params.p16 = helper::param2Fixpoint(config.p16);
  params.p17 = helper::param2Fixpoint(config.p17);
  params.p18 = helper::param2Fixpoint(config.p18);
  params.p19 = helper::param2Fixpoint(config.p19);
  params.p20 = helper::param2Fixpoint(config.p20);
  params.p21 = helper::param2Fixpoint(config.p21);
  params.p22 = helper::param2Fixpoint(config.p22);
  params.p23 = helper::param2Fixpoint(config.p23);
  params.p24 = helper::param2Fixpoint(config.p24);
  params.p25 = helper::param2Fixpoint(config.p25);
  params.p26 = helper::param2Fixpoint(config.p26);
  params.p27 = helper::param2Fixpoint(config.p27);
  params.p28 = helper::param2Fixpoint(config.p28);
  params.p29 = helper::param2Fixpoint(config.p29);
  params.p30 = helper::param2Fixpoint(config.p30);
  params.p31 = helper::param2Fixpoint(config.p31);
  params.p32 = helper::param2Fixpoint(config.p32);
  params.p33 = helper::param2Fixpoint(config.p33);
  params.p34 = helper::param2Fixpoint(config.p34);
  params.p35 = helper::param2Fixpoint(config.p35);
  params.p36 = helper::param2Fixpoint(config.p36);
  params.p37 = helper::param2Fixpoint(config.p37);
  params.p38 = helper::param2Fixpoint(config.p38);
  params.p39 = helper::param2Fixpoint(config.p39);
  params.p40 = helper::param2Fixpoint(config.p40);
  params.p41 = helper::param2Fixpoint(config.p41);
  params.p42 = helper::param2Fixpoint(config.p42);
  params.p43 = helper::param2Fixpoint(config.p43);
  params.p44 = helper::param2Fixpoint(config.p44);
  params.p45 = helper::param2Fixpoint(config.p45);
  params.p46 = helper::param2Fixpoint(config.p46);
  params.p47 = helper::param2Fixpoint(config.p47);
  params.p48 = helper::param2Fixpoint(config.p48);
  params.p49 = helper::param2Fixpoint(config.p49);
  params.p50 = helper::param2Fixpoint(config.p50);

  for(int i=0; i<5; i++){
    if(comm_->sendPacketAck(HLI_PACKET_ID_SSDK_PARAMETERS, params)){
      ROS_INFO("sent SSDK parameters");
      return true;
    }
  }

  ROS_ERROR("sending SSDK parameters failed, tried 5 times");
  return false;
}
