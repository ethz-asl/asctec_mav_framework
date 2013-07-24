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

#include "hl_interface.h"
#include "helper.h"
#include <asctec_hl_comm/MotorSpeed.h>

HLInterface::HLInterface(ros::NodeHandle & nh, CommPtr & comm) :
  nh_(nh), pnh_("~/fcu"), comm_(comm), gps_status_(sensor_msgs::NavSatStatus::STATUS_NO_FIX), gps_satellites_used_(0),
      diag_updater_(), diag_imu_freq_(diagnostic_updater::FrequencyStatusParam(&diag_imu_freq_min_,
                                                                               &diag_imu_freq_max_, 0, 5))
{
  pnh_.param("frame_id", frame_id_, std::string("fcu"));
  pnh_.param("k_stick", k_stick_, 25);
  pnh_.param("k_stick_yaw", k_stick_yaw_, 120);
  pnh_.param("stddev_angular_velocity", angular_velocity_variance_, 0.013); // taken from experiments
  pnh_.param("stddev_linear_acceleration", linear_acceleration_variance_, 0.083); // taken from experiments
  angular_velocity_variance_ *= angular_velocity_variance_;
  linear_acceleration_variance_ *= linear_acceleration_variance_;

  imu_pub_ = nh_.advertise<asctec_hl_comm::mav_imu> ("imu_custom", 1);
  imu_ros_pub_ = nh_.advertise<sensor_msgs::Imu> ("imu", 1);
  motors_pub_ = nh_.advertise<asctec_hl_comm::MotorSpeed> ("motor_speed", 1);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix> ("gps", 1);
  gps_custom_pub_ = nh_.advertise<asctec_hl_comm::GpsCustom> ("gps_custom", 1);
  rc_pub_ = nh_.advertise<asctec_hl_comm::mav_rcdata> ("rcdata", 1);
  status_pub_ = nh_.advertise<asctec_hl_comm::mav_status> ("status", 1);
  mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped> ("mag", 1);

  control_sub_ = nh_.subscribe("control", 1, &HLInterface::controlCmdCallback, this);

  motor_srv_ = nh_.advertiseService("motor_control", &HLInterface::cbMotors, this);
  crtl_srv_ = nh_.advertiseService("control", &HLInterface::cbCtrl, this);

  config_ = asctec_hl_interface::HLInterfaceConfig::__getDefault__();

  // bring up dynamic reconfigure
  reconf_srv_ = new ReconfigureServer(pnh_);
  ReconfigureServer::CallbackType f = boost::bind(&HLInterface::cbConfig, this, _1, _2);
  reconf_srv_->setCallback(f);

  diag_updater_.add("AscTec AutoPilot status", this, &HLInterface::diagnostic);
  diag_updater_.setHardwareID("none");
  diag_updater_.add(diag_imu_freq_);
  diag_updater_.force_update();

  // register callbacks for packets from serial port
  comm_->registerCallback(HLI_PACKET_ID_IMU, &HLInterface::processImuData, this);
  comm_->registerCallback(HLI_PACKET_ID_RC, &HLInterface::processRcData, this);
  comm_->registerCallback(HLI_PACKET_ID_GPS, &HLInterface::processGpsData, this);
  comm_->registerCallback(HLI_PACKET_ID_STATUS, &HLInterface::processStatusData, this);
  comm_->registerCallback(HLI_PACKET_ID_TIMESYNC, &HLInterface::processTimeSyncData, this);
  comm_->registerCallback(HLI_PACKET_ID_MAG, &HLInterface::processMagData, this);
}

HLInterface::~HLInterface()
{
  delete reconf_srv_;
}

void HLInterface::processImuData(uint8_t * buf, uint32_t bufLength)
{
  HLI_IMU* data = (HLI_IMU*)buf;
  static int seq = 0;
  diag_imu_freq_.tick();

  double roll = helper::asctecAttitudeToSI(data->ang_roll);
  double pitch = helper::asctecAttitudeToSI(data->ang_pitch);
  double yaw = helper::asctecAttitudeToSI(data->ang_yaw);

  if (yaw > M_PI)
    yaw -= 2 * M_PI;

  double height = data->height * 0.001;
  double differential_height = data->differential_height * 0.001;

  uint32_t subs_imu = imu_pub_.getNumSubscribers();
  uint32_t subs_imu_ros = imu_ros_pub_.getNumSubscribers();

  if (subs_imu > 0 || subs_imu_ros > 0)
  {
    geometry_msgs::Quaternion q;
    helper::angle2quaternion(roll, pitch, yaw, &q.w, &q.x, &q.y, &q.z);

    if (subs_imu > 0)
    {
      asctec_hl_comm::mav_imuPtr msg(new asctec_hl_comm::mav_imu);

      msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
      msg->header.seq = seq;
      msg->header.frame_id = frame_id_;
      msg->acceleration.x = helper::asctecAccToSI(data->acc_x);
      msg->acceleration.y = helper::asctecAccToSI(data->acc_y);
      msg->acceleration.z = helper::asctecAccToSI(data->acc_z);
      msg->angular_velocity.x = helper::asctecOmegaToSI(data->ang_vel_roll);
      msg->angular_velocity.y = helper::asctecOmegaToSI(data->ang_vel_pitch);
      msg->angular_velocity.z = helper::asctecOmegaToSI(data->ang_vel_yaw);
      msg->differential_height = differential_height;
      msg->height = height;
      msg->orientation = q;

      imu_pub_.publish(msg);
    }
    if (subs_imu_ros > 0)
    {
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
      msg->header.seq = seq;
      msg->header.frame_id = frame_id_;
      msg->linear_acceleration.x = helper::asctecAccToSI(data->acc_x);
      msg->linear_acceleration.y = helper::asctecAccToSI(data->acc_y);
      msg->linear_acceleration.z = helper::asctecAccToSI(data->acc_z);
      msg->angular_velocity.x = helper::asctecOmegaToSI(data->ang_vel_roll);
      msg->angular_velocity.y = helper::asctecOmegaToSI(data->ang_vel_pitch);
      msg->angular_velocity.z = helper::asctecOmegaToSI(data->ang_vel_yaw);
      msg->orientation = q;
      helper::setDiagonalCovariance(msg->angular_velocity_covariance, angular_velocity_variance_);
      helper::setDiagonalCovariance(msg->linear_acceleration_covariance, linear_acceleration_variance_);

      imu_ros_pub_.publish(msg);
    }
  }

  asctec_hl_comm::MotorSpeedPtr msg_motors (new asctec_hl_comm::MotorSpeed);
  msg_motors->header.stamp = ros::Time(data->timestamp * 1.0e-6);
  msg_motors->header.seq = seq;
  msg_motors->header.frame_id = frame_id_;
  for(int i = 0; i<6; ++i)
    msg_motors->motor_speed[i] = data->motors[i];

  motors_pub_.publish(msg_motors);

  seq++;
}

void HLInterface::processGpsData(uint8_t * buf, uint32_t bufLength)
{
  HLI_GPS* data = (HLI_GPS*)buf;
  static int seq = 0;
  sensor_msgs::NavSatFixPtr gps_fix(new sensor_msgs::NavSatFix);
  asctec_hl_comm::GpsCustomPtr gps_custom(new asctec_hl_comm::GpsCustom);

  gps_fix->header.stamp = ros::Time(((double)data->timestamp) * 1.0e-6);
  gps_fix->header.seq = seq;
  gps_fix->header.frame_id = frame_id_;
  gps_fix->latitude = static_cast<double> (data->latitude) * 1.0e-7;
  gps_fix->longitude = static_cast<double> (data->longitude) * 1.0e-7;
  gps_fix->altitude = static_cast<double> (data->height) * 1.0e-3;

  // TODO: check covariance
  double var_h = static_cast<double> (data->horizontalAccuracy) * 1.0e-3 / 3.0; // accuracy, 3 sigma bound ???
  double var_v = static_cast<double> (data->verticalAccuracy) * 1.0e-3 / 3.0;
  var_h *= var_h;
  var_v *= var_v;

  gps_fix->position_covariance[0] = var_h;
  gps_fix->position_covariance[4] = var_h;
  gps_fix->position_covariance[8] = var_v;
  gps_fix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  gps_fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  if (data->status & 0x01)
    gps_fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  else
    gps_fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

  gps_custom->header = gps_fix->header;
  gps_custom->status = gps_fix->status;
  gps_custom->longitude = gps_fix->longitude;
  gps_custom->latitude = gps_fix->latitude;
  gps_custom->altitude = gps_fix->altitude;
  gps_custom->position_covariance = gps_fix->position_covariance;
  gps_custom->position_covariance_type = gps_fix->position_covariance_type;

  gps_custom->velocity_x = static_cast<double> (data->speedX) * 1.0e-3;
  gps_custom->velocity_y = static_cast<double> (data->speedY) * 1.0e-3;

  gps_custom->pressure_height = static_cast<double>(data->pressure_height) * 1.0e-3;

  // TODO: check covariance
  double var_vel = static_cast<double> (data->speedAccuracy) * 1.0e-3 / 3.0; // accuracy, 3 sigma bound ???
  var_vel *= var_vel;
  gps_custom->velocity_covariance[0] = gps_custom->velocity_covariance[3] = var_vel;

  // copy this for the other status message
  gps_status_ = gps_fix->status.status;
  gps_satellites_used_ = data->numSatellites;

  seq++;
  gps_pub_.publish(gps_fix);
  gps_custom_pub_.publish(gps_custom);
}

void HLInterface::processRcData(uint8_t * buf, uint32_t bufLength)
{
  HLI_RCDATA* data = (HLI_RCDATA*)buf;
  static int seq = 0;
  asctec_hl_comm::mav_rcdataPtr msg(new asctec_hl_comm::mav_rcdata);

  msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
  msg->header.frame_id = frame_id_;
  msg->header.seq = seq;

  size_t n_channels = std::min(msg->channel.size(), size_t(HLI_NUMBER_RC_CHANNELS));

  for (size_t i = 0; i < n_channels; i++)
    msg->channel[i] = data->channel[i];

  seq++;
  rc_pub_.publish(msg);
}

void HLInterface::processStatusData(uint8_t * buf, uint32_t bufLength)
{
  HLI_STATUS* data = (HLI_STATUS*)buf;
  static int seq = 0;
  asctec_hl_comm::mav_statusPtr msg(new asctec_hl_comm::mav_status);

  msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
  msg->header.frame_id = frame_id_;
  msg->header.seq = seq;

  msg->battery_voltage = data->battery_voltage * 0.001;
  msg->flight_time = data->flight_time;
  msg->debug1 = data->debug1;
  msg->debug2 = data->debug2;
  msg->cpu_load = data->cpu_load;

  switch (data->motors)
  {
    case -1:
      msg->motor_status = "off";
      break;
    case 0:
      msg->motor_status = "stopping";
      break;
    case 1:
      msg->motor_status = "starting";
      break;
    case 2:
      msg->motor_status = "running";
      break;
    default:
      msg->motor_status = "";
      break;
  }

  if ((data->flight_mode & 0x0F) == HLI_FLIGHTMODE_ACC)
    msg->flight_mode_ll = "Acc";
  else if ((data->flight_mode & 0x0F) == HLI_FLIGHTMODE_HEIGHT)
    msg->flight_mode_ll = "Height";
  else if ((data->flight_mode & 0x0F) == HLI_FLIGHTMODE_GPS)
    msg->flight_mode_ll = "GPS";
  else
    msg->flight_mode_ll = "unknown";

  msg->serial_interface_enabled = data->flight_mode & HLI_SERIALINTERFACE_ENABLED_BIT;
  msg->serial_interface_active = data->flight_mode & HLI_SERIALINTERFACE_ACTIVE_BIT;

  switch (gps_status_)
  {
    case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
      msg->gps_status = "GPS no fix";
      break;
    case sensor_msgs::NavSatStatus::STATUS_FIX:
      msg->gps_status = "GPS fix";
      break;
    case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
      msg->gps_status = "SBAS fix";
      break;
    case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
      msg->gps_status = "GBAS fix";
      break;
    default:
      msg->gps_status = "";
      break;
  }

  msg->gps_num_satellites = gps_satellites_used_;

  msg->have_SSDK_parameters = data->have_SSDK_parameters == 1;

  switch (data->state_estimation)
  {
    case HLI_MODE_STATE_ESTIMATION_EXT:
      msg->state_estimation = asctec_hl_interface::HLInterface_STATE_EST_EXTERN;
      break;
    case HLI_MODE_STATE_ESTIMATION_HL_EKF:
      msg->state_estimation = asctec_hl_interface::HLInterface_STATE_EST_HIGHLEVEL_EKF;
      break;
    case HLI_MODE_STATE_ESTIMATION_HL_SSDK:
      msg->state_estimation = asctec_hl_interface::HLInterface_STATE_EST_HIGHLEVEL_SSDK;
      break;
    case HLI_MODE_STATE_ESTIMATION_OFF:
      msg->state_estimation = asctec_hl_interface::HLInterface_STATE_EST_OFF;
      break;
    default:
      msg->state_estimation = "unknown";
      break;
  }

  switch (data->position_control)
  {
    case HLI_MODE_POSCTRL_HL:
      msg->position_control = asctec_hl_interface::HLInterface_POSCTRL_HIGHLEVEL;
      break;
    case HLI_MODE_POSCTRL_LL:
      msg->position_control = asctec_hl_interface::HLInterface_POSCTRL_GPS;
      break;
    case HLI_MODE_POSCTRL_OFF:
      msg->position_control = asctec_hl_interface::HLInterface_POSCTRL_OFF;
      break;
    default:
      msg->position_control = "unknown";
      break;
  }

  msg->rx_packets = comm_->getRxPackets();
  msg->rx_packets_good = comm_->getRxPacketsGood();
  msg->tx_packets = data->rx_packets;
  msg->tx_packets_good = data->rx_packets_good;

  msg->timesync_offset = (float)data->timesync_offset * 1e-6;

  seq++;
  status_ = *msg;
  status_pub_.publish(msg);

  diag_updater_.update();
}


void HLInterface::diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  unsigned char summary_level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string summary_message = "OK";

  // TODO: are these thresholds ok?

  if (status_.battery_voltage < (config_.battery_warning - 0.5))
  {
    summary_level = diagnostic_msgs::DiagnosticStatus::ERROR;
    summary_message = "battery voltage critical - land now !!!";
    ROS_ERROR_STREAM_THROTTLE(1, "" << summary_message << ": " << status_.battery_voltage << " V");
  }
  else if (status_.battery_voltage < config_.battery_warning)
  {
    summary_level = diagnostic_msgs::DiagnosticStatus::WARN;
    summary_message = "battery voltage low";
    ROS_WARN_STREAM_THROTTLE(1, "" << summary_message << ": " << status_.battery_voltage << " V");
  }

  stat.summary(summary_level, summary_message);
  stat.add("Battery voltage [V]", status_.battery_voltage);
  stat.add("Flight time [s]", status_.flight_time);
  stat.add("HLP cpu load [%]", status_.cpu_load);
  stat.add("Position control mode", status_.position_control);
  stat.add("State estimation mode", status_.state_estimation);
  stat.add("GPS status", status_.gps_status);
  stat.add("GPS # satellites", status_.gps_num_satellites);
  stat.add("Flight mode LLP", status_.flight_mode_ll);
  stat.add("Motor status", status_.motor_status);
  stat.add("Successful TX packets [%]", (float)status_.tx_packets_good/(float)status_.tx_packets*100.0);
  stat.add("Successful RX packets [%]", (float)status_.rx_packets_good/(float)status_.rx_packets*100.0);
  stat.add("Serial interface enabled", static_cast<bool>(status_.serial_interface_enabled));
  stat.add("Serial interface active", static_cast<bool>(status_.serial_interface_active));
  stat.add("Have SSDK parameters", static_cast<bool>(status_.have_SSDK_parameters));
  stat.add("Timesync offset [ms]", status_.timesync_offset*1000.0);
  stat.add("Debug 1", status_.debug1);
  stat.add("Debug 2", status_.debug2);
}

void HLInterface::processTimeSyncData(uint8_t * buf, uint32_t bufLength)
{
  HLI_TIMESYNC data = *(HLI_TIMESYNC*)buf;

  data.ts1 = (uint64_t)(ros::Time::now().toSec() * 1.0e6);

  comm_->sendPacket(HLI_PACKET_ID_TIMESYNC, data);

  // warn if imu time is too far away from pc time. At low baudrates, the IMU will take longer to sync.
  ROS_WARN_STREAM_COND(std::abs(status_.timesync_offset) > 0.002, "imu time is off by "<<status_.timesync_offset * 1e3 <<" ms");
}

void HLInterface::processMagData(uint8_t * buf, uint32_t bufLength)
{
  HLI_MAG* data = (HLI_MAG*)buf;
  static int seq = 0;
  geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);

  msg->header.stamp = ros::Time(data->timestamp * 1.0e-6);
  msg->header.frame_id = frame_id_;
  msg->header.seq = seq;

  msg->vector.x = data->x;
  msg->vector.y = data->y;
  msg->vector.z = data->z;

  seq++;
  mag_pub_.publish(msg);
}

bool HLInterface::cbMotors(asctec_hl_comm::mav_ctrl_motors::Request &req,
                           asctec_hl_comm::mav_ctrl_motors::Response &resp)
{
  HLI_MOTORS data;

  if (req.startMotors)
  {
    data.motors = 1;
    ROS_INFO("request to start motors");
  }
  else
  {
    data.motors = 0;
    ROS_INFO("request to stop motors");
  }

  // make sure packet arrives
  bool success = false;
  for (int i = 0; i < 5; i++)
  {
    success = comm_->sendPacketAck(HLI_PACKET_ID_MOTORS, data, 0.5);
    if (success)
      break;
  }

  if(!success)
  {
    ROS_WARN("unable to send motor on/off command");
    resp.motorsRunning = status_.motor_status == "running";
    return false;
  }

  ros::Duration d(0.1);
  for (int i = 0; i < 10; i++)
  {
    if (req.startMotors && status_.motor_status == "running")
    {
      resp.motorsRunning = true;
      return true;
    }
    else if (!req.startMotors && status_.motor_status == "off")
    {
      resp.motorsRunning = false;
      return true;
    }
    d.sleep();
  }

  ROS_WARN("switching motors %s timed out", req.startMotors?"on":"off");
  return false;
}

void HLInterface::controlCmdCallback(const asctec_hl_comm::mav_ctrlConstPtr & msg)
{
  sendControlCmd(*msg);
}

bool HLInterface::cbCtrl(asctec_hl_comm::MavCtrlSrv::Request & req, asctec_hl_comm::MavCtrlSrv::Response & resp)
{
  sendControlCmd(req.ctrl, &resp.ctrl_result);
  return resp.ctrl_result.type != -1;
}

void HLInterface::sendControlCmd(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result)
{
  bool validCommand = false;

  if (ctrl.type == asctec_hl_comm::mav_ctrl::acceleration)
  {
    if (config_.position_control == asctec_hl_interface::HLInterface_POSCTRL_OFF)
    {
      sendAccCommandLL(ctrl, ctrl_result);
      validCommand = true;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(2,
          "GPS/Highlevel position control must be turned off. "
          "Set \"position_control\" parameter to \"off\"");
      if(ctrl_result != NULL)
        ctrl_result->type = -1;
    }
  }

  else if (ctrl.type == asctec_hl_comm::mav_ctrl::velocity_body)
  {
    if (config_.position_control == asctec_hl_interface::HLInterface_POSCTRL_GPS)
    {
      sendVelCommandLL(ctrl, ctrl_result);
      validCommand = true;
    }
    else if (config_.position_control == asctec_hl_interface::HLInterface_POSCTRL_HIGHLEVEL)
    {
      sendVelCommandHL(ctrl, ctrl_result);
      validCommand = true;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(2,
          "Higlevel or Lowlevel processor position control has not "
          "been chosen. Set \"position_control\" parameter to "
          "\"HighLevel\" or \"GPS\" ! sending nothing to mav !");
      if(ctrl_result != NULL)
        ctrl_result->type = -1;
    }
  }
  else if (ctrl.type == asctec_hl_comm::mav_ctrl::velocity)
    {
      if (config_.position_control == asctec_hl_interface::HLInterface_POSCTRL_HIGHLEVEL)
      {
        sendVelCommandHL(ctrl, ctrl_result);
        validCommand = true;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(2,
            "Higlevel or Lowlevel processor position control has not "
            "been chosen. Set \"position_control\" parameter to "
            "\"HighLevel\" ! sending nothing to mav !");
        if(ctrl_result != NULL)
          ctrl_result->type = -1;
      }
    }
  else if (ctrl.type == asctec_hl_comm::mav_ctrl::position || ctrl.type == asctec_hl_comm::mav_ctrl::position_body)
  {
    // allow to "inherit" max velocity from parameters
    asctec_hl_comm::mav_ctrl ctrl_msg = ctrl;
    if(ctrl_msg.v_max_xy == -1)
      ctrl_msg.v_max_xy = config_.max_velocity_xy;

    if(ctrl_msg.v_max_z == -1)
      ctrl_msg.v_max_z = config_.max_velocity_z;

    if (config_.position_control == asctec_hl_interface::HLInterface_POSCTRL_HIGHLEVEL &&
        config_.state_estimation != asctec_hl_interface::HLInterface_STATE_EST_OFF)
    {
      sendPosCommandHL(ctrl_msg, ctrl_result);
      validCommand = true;
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(2,
          "Higlevel processor position control was not chosen and/or no state "
          "estimation was selected. Set the \"position_control\" parameter to "
          "\"HighLevel\" and the \"state_estimation\" parameter to anything but \"off\"! sending nothing to mav !");
      if(ctrl_result != NULL)
        ctrl_result->type = -1;
    }
  }

  // was the controlmode specified properly?
  if (!validCommand)
  {
    ROS_WARN_STREAM_THROTTLE(2,"Control type was not specified ... not sending anything to the mav");
  }
}

void HLInterface::sendAccCommandLL(const asctec_hl_comm::mav_ctrl & msg, asctec_hl_comm::mav_ctrl * ctrl_result)
{
  HLI_CMD_LL ctrlLL;

  // spin-directions positive according to right hand rule around axis
  ctrlLL.x = helper::clamp<short>(-2047, 2047, (short)(msg.x * 180.0 / M_PI * 1000.0 / (float)k_stick_)); // cmd=real_angle*1000/K_stick
  ctrlLL.y = helper::clamp<short>(-2047, 2047, (short)(msg.y * 180.0 / M_PI * 1000.0 / (float)k_stick_)); // dito

  // cmd=real_anglular_velocity*1000/k_stick_yaw,
  // cmd is limited to +- 1700 such that it's not possible to switch the motors with a bad command
  ctrlLL.yaw = helper::clamp<short>(-1700, 1700, (short)(msg.yaw * 180.0 / M_PI * 1000.0 / (float)k_stick_yaw_));

  // catch wrong thrust command ;-)
  if (msg.z > 1.0)
  {
    ROS_ERROR(
        "I just prevented you from giving full thrust..."
        "\nset the input range correct!!! [0 ... 1.0] ");

    if(ctrl_result != NULL)
      ctrl_result->type = -1;

    return;
  }
  else
  {
    ctrlLL.z = helper::clamp<short>(0, 4096, (short)(msg.z * 4096.0));
  }

  if(ctrl_result != NULL)
  {
    *ctrl_result = msg;
    ctrl_result->x = static_cast<float>(ctrlLL.x * k_stick_) / 180.0 * M_PI * 1e-3;
    ctrl_result->y = static_cast<float>(ctrlLL.y * k_stick_) / 180.0 * M_PI * 1e-3;
    ctrl_result->z = static_cast<float>(ctrlLL.z) / 4096.0;
    ctrl_result->yaw = static_cast<float>(ctrlLL.yaw * k_stick_yaw_) / 180.0 * M_PI * 1e-3;
  }

//  ROS_INFO_STREAM("sending command: x:"<<ctrlLL.x<<" y:"<<ctrlLL.y<<" yaw:"<<ctrlLL.yaw<<" z:"<<ctrlLL.z<<" ctrl:"<<enable_ctrl_);
  comm_->sendPacket(HLI_PACKET_ID_CONTROL_LL, ctrlLL);
}

void HLInterface::sendVelCommandLL(const asctec_hl_comm::mav_ctrl & msg, asctec_hl_comm::mav_ctrl * ctrl_result)
{
  HLI_CMD_LL ctrlLL;

  ctrlLL.x = helper::clamp<short>(-2047, 2047, (short)(msg.x / config_.max_velocity_xy * 2047.0));
  ctrlLL.y = helper::clamp<short>(-2047, 2047, (short)(msg.y / config_.max_velocity_xy * 2047.0));
  ctrlLL.yaw = helper::clamp<short>(-2047, 2047, (short)(msg.yaw / config_.max_velocity_yaw* 2047.0));
  ctrlLL.z = helper::clamp<short>(-2047, 2047, (short)(msg.z / config_.max_velocity_z * 2047.0)) + 2047; // "zero" is still 2047!

  if (ctrl_result != NULL)
  {
    *ctrl_result = msg;
    ctrl_result->x = static_cast<float> (ctrlLL.x) / 2047.0 * config_.max_velocity_xy;
    ctrl_result->y = static_cast<float> (ctrlLL.y) / 2047.0 * config_.max_velocity_xy;
    ctrl_result->z = static_cast<float> (ctrlLL.z - 2047) / 2047.0 * config_.max_velocity_z;
    ctrl_result->yaw = static_cast<float> (ctrlLL.yaw) / 2047.0 * config_.max_velocity_yaw;
  }

//  ROS_INFO_STREAM("sending command: x:"<<ctrlLL.x<<" y:"<<ctrlLL.y<<" yaw:"<<ctrlLL.yaw<<" z:"<<ctrlLL.z<<" ctrl:"<<enable_ctrl_);
  comm_->sendPacket(HLI_PACKET_ID_CONTROL_LL, ctrlLL);
}

void HLInterface::sendVelCommandHL(const asctec_hl_comm::mav_ctrl & msg, asctec_hl_comm::mav_ctrl * ctrl_result)
{
  HLI_CMD_HL ctrlHL;

  ctrlHL.vX = (short)(helper::clamp<float>(-config_.max_velocity_xy, config_.max_velocity_xy, msg.x) * 1000.0);
  ctrlHL.vY = (short)(helper::clamp<float>(-config_.max_velocity_xy, config_.max_velocity_xy, msg.y) * 1000.0);
  ctrlHL.vZ = (short)(helper::clamp<float>(-config_.max_velocity_z, config_.max_velocity_z, msg.z) * 1000.0);
  ctrlHL.vYaw = (short)(helper::clamp<float>(-config_.max_velocity_yaw, config_.max_velocity_yaw, msg.yaw) * 180.0 / M_PI * 1000.0);

  ctrlHL.x = 0;
  ctrlHL.y = 0;
  ctrlHL.z = 0;
  ctrlHL.heading = 0;

  ctrlHL.bitfield = 1;
  if(msg.type == asctec_hl_comm::mav_ctrl::velocity_body)
    ctrlHL.bitfield |= EXT_POSITION_CMD_BODYFIXED;

  if (ctrl_result != NULL)
  {
    *ctrl_result = msg;
    ctrl_result->x = static_cast<float> (ctrlHL.vX) * 1e-3;
    ctrl_result->y = static_cast<float> (ctrlHL.vY) * 1e-3;
    ctrl_result->z = static_cast<float> (ctrlHL.vZ) * 1e-3;
    ctrl_result->yaw = static_cast<float> (ctrlHL.vYaw) / 180.0e3 * M_PI;
  }

  comm_->sendPacket(HLI_PACKET_ID_CONTROL_HL, ctrlHL);
}
void HLInterface::sendPosCommandHL(const asctec_hl_comm::mav_ctrl & msg, asctec_hl_comm::mav_ctrl * ctrl_result)
{
  HLI_CMD_HL ctrlHL;
  static unsigned int seq = 1; // <-- set to one, otherwise first packet doesn't get through

  if (std::abs(msg.yaw) > M_PI)
  {
    ROS_WARN("yaw has to be in [-pi ... pi], got %f instead", msg.yaw);
    if (ctrl_result != NULL)
      ctrl_result->type = -1;
    return;
  }

  ctrlHL.seq = seq;
  ctrlHL.vX = 0;
  ctrlHL.vX = 0;
  ctrlHL.vZ = 0;
  //			ctrlHL.vHeading = 0;

  ctrlHL.x = static_cast<int>(helper::clamp<float>(config_.min_pos_x, config_.max_pos_x, msg.x) * 1000.0);
  ctrlHL.y = static_cast<int>(helper::clamp<float>(config_.min_pos_y, config_.max_pos_y, msg.y) * 1000.0);
  ctrlHL.z = static_cast<int>(helper::clamp<float>(config_.min_pos_z, config_.max_pos_z, msg.z) * 1000.0);
  // asctec uses 0...360° * 1000, we -pi...+pi
  ctrlHL.heading = helper::yaw2asctec(msg.yaw);

  ctrlHL.vMaxXY = static_cast<short>(std::min<float>(config_.max_velocity_xy, msg.v_max_xy)*1000);
  ctrlHL.vMaxZ = static_cast<short>(std::min<float>(config_.max_velocity_z, msg.v_max_z)*1000);

  ctrlHL.bitfield = 0;

  if(msg.type == asctec_hl_comm::mav_ctrl::position_body)
    ctrlHL.bitfield |= EXT_POSITION_CMD_BODYFIXED;

  if (ctrl_result != NULL)
  {
    *ctrl_result = msg;
    ctrl_result->x = static_cast<float> (ctrlHL.x) * 1e-3;
    ctrl_result->y = static_cast<float> (ctrlHL.y) * 1e-3;
    ctrl_result->z = static_cast<float> (ctrlHL.z) * 1e-3;
    ctrl_result->yaw = static_cast<float> (ctrlHL.heading) / 180.0e3 * M_PI;
    if (ctrl_result->yaw > M_PI)
        ctrl_result->yaw -= 2 * M_PI;
  }

  comm_->sendPacket(HLI_PACKET_ID_CONTROL_HL, ctrlHL);
  seq++;
}

void HLInterface::cbConfig(asctec_hl_interface::HLInterfaceConfig & config, uint32_t level)
{

  if (level & asctec_hl_interface::HLInterface_HLI_CONFIG)
  {
    /** bits for the control byte:
     * bit 0: pitch control enabled
     * bit 1: roll control enabled
     * bit 2: yaw control enabled
     * bit 3: thrust control enabled
     * bit 4: Height control enabled
     * bit 5: GPS position control enabled
     */

    enable_ctrl_ = 0;
    if (config.enable_x)
      enable_ctrl_ |= 1;
    if (config.enable_y)
      enable_ctrl_ |= 1 << 1;
    if (config.enable_yaw)
      enable_ctrl_ |= 1 << 2;
    if (config.enable_z)
      enable_ctrl_ |= 1 << 3;
    if (config.position_control == asctec_hl_interface::HLInterface_POSCTRL_GPS)
    {
      enable_ctrl_ |= 1 << 4;
      enable_ctrl_ |= 1 << 5;
    }

    HLI_CONFIG cfg;
    cfg.position_control_axis_enable = enable_ctrl_;

    if (config.position_control == asctec_hl_interface::HLInterface_POSCTRL_HIGHLEVEL)
      cfg.mode_position_control = HLI_MODE_POSCTRL_HL;
    else if (config.position_control == asctec_hl_interface::HLInterface_POSCTRL_GPS)
      cfg.mode_position_control = HLI_MODE_POSCTRL_LL;
    else
      cfg.mode_position_control = HLI_MODE_POSCTRL_OFF;

    if (config.state_estimation == asctec_hl_interface::HLInterface_STATE_EST_EXTERN)
      cfg.mode_state_estimation = HLI_MODE_STATE_ESTIMATION_EXT;
    else if (config.state_estimation == asctec_hl_interface::HLInterface_STATE_EST_HIGHLEVEL_SSDK)
      cfg.mode_state_estimation = HLI_MODE_STATE_ESTIMATION_HL_SSDK;
    else if (config.state_estimation == asctec_hl_interface::HLInterface_STATE_EST_HIGHLEVEL_EKF)
      cfg.mode_state_estimation = HLI_MODE_STATE_ESTIMATION_HL_EKF;
    else
      cfg.mode_state_estimation = HLI_MODE_STATE_ESTIMATION_OFF;

    cfg.battery_warning_voltage = static_cast<uint16_t>(config.battery_warning * 1000); // convert to mV

    if(!comm_->sendPacketAck(HLI_PACKET_ID_CONFIG, cfg)){
      config.enable_x = config_.enable_x;
      config.enable_y = config_.enable_y;
      config.enable_z = config_.enable_z;
      config.enable_yaw = config_.enable_yaw;
      config.position_control = config_.position_control;
      config.state_estimation = config_.state_estimation;
      config.battery_warning = config_.battery_warning;
    }
  }

  if (level & asctec_hl_interface::HLInterface_PACKET_RATE)
  {
    HLI_SUBSCRIPTION ps;
    ps.imu = helper::rateToPeriod(config.packet_rate_imu);
    ps.rcdata = helper::rateToPeriod(config.packet_rate_rc);
    ps.gps = helper::rateToPeriod(config.packet_rate_gps);
    ps.ssdk_debug = helper::rateToPeriod(config.packet_rate_ssdk_debug);
    ps.ekf_state = helper::rateToPeriod(config.packet_rate_ekf_state);
    ps.mag = helper::rateToPeriod(config.packet_rate_mag);

    diag_imu_freq_min_ = 0.95 * config.packet_rate_imu;
    diag_imu_freq_max_ = 1.05 * config.packet_rate_imu;

    if(!comm_->sendPacketAck(HLI_PACKET_ID_SUBSCRIPTION, ps)){
      config.packet_rate_imu = config_.packet_rate_imu;
      config.packet_rate_rc = config_.packet_rate_rc;
      config.packet_rate_gps = config_.packet_rate_gps;
      config.packet_rate_ssdk_debug = config_.packet_rate_ssdk_debug;
      config.packet_rate_ekf_state = config_.packet_rate_ekf_state;
      config.packet_rate_mag = config_.packet_rate_mag;
    }
  }

  config_ = config;
}


