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

#ifndef ASCTECINTERFACE_H_
#define ASCTECINTERFACE_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "comm.h"

// message includes
#include <asctec_hl_comm/mav_rcdata.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/mav_status.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

// service includes
#include <asctec_hl_comm/MavCtrlSrv.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>

// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_hl_interface/HLInterfaceConfig.h>

typedef dynamic_reconfigure::Server<asctec_hl_interface::HLInterfaceConfig> ReconfigureServer;

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

/*
 * This class provides basic access to the HighLevel Processor of the AutoPilot Board. It subscribes to position / velocity / acceleration commands, publishes IMU, GPS, RC and status data and synchronizes the HL Processor time with the host computer.
 * HighLevel Processor and LowLevel Processor of the AscTec AutoPilot board are referenced to HLP and LLP in the following.
 */

class HLInterface
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  CommPtr & comm_;

  std::string frame_id_;

  ros::Publisher gps_pub_;
  ros::Publisher gps_custom_pub_;
  ros::Publisher imu_ros_pub_; ///< publisher for sensor_msgs/Imu message
  ros::Publisher imu_pub_; ///< publisher for custom asctec_hl_comm/mav_imu message
  ros::Publisher motors_pub_; ///< publisher for motor message
  ros::Publisher rc_pub_;
  ros::Publisher status_pub_;
  ros::Publisher mag_pub_;
  ros::Subscriber control_sub_;

  ros::ServiceServer motor_srv_;
  ros::ServiceServer crtl_srv_;

  // callback functions for data from the serial port
  void processImuData(uint8_t * buf, uint32_t bufLength);
  void processGpsData(uint8_t * buf, uint32_t bufLength);
  void processRcData(uint8_t * buf, uint32_t bufLength);
  void processStatusData(uint8_t * buf, uint32_t bufLength);
  void processTimeSyncData(uint8_t * buf, uint32_t bufLength);
  void processPoseEKFData(uint8_t * buf, uint32_t bufLength);
  void processMagData(uint8_t * buf, uint32_t bufLength);

  /// service to start/stop motors
  bool cbMotors(asctec_hl_comm::mav_ctrl_motors::Request &req, asctec_hl_comm::mav_ctrl_motors::Response &resp);

  /// ctrl service callback
  bool cbCtrl(asctec_hl_comm::MavCtrlSrv::Request & req, asctec_hl_comm::MavCtrlSrv::Response & resp);

  /**
   * callback that listens to mav_ctrl messages
   * message must be sent at least at 10 Hz, otherwise the mav will switch back to manual control!
   **/
  void controlCmdCallback(const asctec_hl_comm::mav_ctrlConstPtr &msg);

  /// evaluates the mav_ctrl message and sends the appropriate commands to the HLP
  void sendControlCmd(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result=NULL);

  /// sends an acceleration command (pitch, roll, thrust), yaw velocity to the LL processor
  void sendAccCommandLL(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result=NULL);

  /// sends a velocity command to the HLP. Position control on the HL has to be enabled
  void sendVelCommandLL(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result=NULL);

  /// sends a velocity command to the LLP. Velocity is controlled based on GPS
  void sendVelCommandHL(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result=NULL);

  /// sends a position (=waypoint) command to the HL. Position control on the HL has to be enabled
  void sendPosCommandHL(const asctec_hl_comm::mav_ctrl & ctrl, asctec_hl_comm::mav_ctrl * ctrl_result=NULL);

  int16_t gps_status_;
  int16_t gps_satellites_used_;

  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  asctec_hl_comm::mav_status status_;

  /// stores the bitmask of degrees of freedom to be controlled
  short enable_ctrl_;

  /// gain from AutoPilot values to 1/1000 degree for the input from the pitch and roll "stick"
  /**
   * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
   * It is only relevant, when this interface is used to send roll/pitch (or x/y velocity when in GPS mode) commands to the LLP.
   */
  int k_stick_;

  /// gain from AutoPilot values to 1/1000 degree for the input from the yaw "stick"
  /**
   * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
   * It is only relevant, when this interface is used to send yaw commands to the LLP.
   */
  int k_stick_yaw_;


  // dynamic reconfigure
  ReconfigureServer *reconf_srv_;
  void cbConfig(asctec_hl_interface::HLInterfaceConfig & config, uint32_t level);
  asctec_hl_interface::HLInterfaceConfig config_;

  // diagnostic update
  diagnostic_updater::Updater diag_updater_;
  diagnostic_updater::FrequencyStatus diag_imu_freq_;
  double diag_imu_freq_min_;
  double diag_imu_freq_max_;
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

public:
  HLInterface(ros::NodeHandle & nh, CommPtr & comm);
  ~HLInterface();
};

#endif /* ASCTECINTERFACE_H_ */
