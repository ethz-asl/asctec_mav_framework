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

#ifndef SSDK_INTERFACE_H_
#define SSDK_INTERFACE_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>

#if ROS_VERSION_MINIMUM(1, 11, 7)
#include <boost/signals2.hpp> // ros indigo
typedef boost::signals2::connection Connection;
#else
#include <boost/signals.hpp> // ros hydro or older
typedef boost::signals::connection Connection;
#endif


#include <tf/transform_listener.h>

// message includes
#include <asctec_hl_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_fusion_comm/ExtState.h>

#include "comm.h"

// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <asctec_hl_interface/SSDKConfig.h>

typedef dynamic_reconfigure::Server<asctec_hl_interface::SSDKConfig> SSDKConfigServer;

///This class provides access to the Simulink SDK on the AscTec Autopilot's HighLevel Processor (HLP) where the data fusion and position controller are implemented.

/**
 * Datafusion is implemented as a Luenberger observer. For position updates, it listens on either a tf transform or a PoseWithCovarinaceStamped message.
 * Position control needs position, velocity as input, which usually comes from the Luenberger Observer. If datafusion is executed somewhere else than the HLP, position and velocity need to be sent (see SSDKInterface::cbState).
 * For position controller and datafusion, parameters can be modified at runtime through dynamic reconfigure through parameter channels and can be monitored through debug channels. See the package documentation for channel assignments.
 */

class SSDKInterface
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  CommPtr & comm_;

  ros::Publisher debug_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber odometry_sub_;

  std::string frame_id_;

  SSDKConfigServer *config_server_;
  asctec_hl_interface::SSDKConfig config_;
  bool have_config_;

  tf::TransformListener tf_listener_;
  Connection tf_callback_;

  /// listens for pose updates via tf
  void tfCallback();

  /// listens on pose updates via PoseWithCovarianceStamped messages.
  void cbPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  /// this callback sends the full state (position, velocity) to the helicopter and bypasses the Luenberger Observer on the HLP
  void cbState(const sensor_fusion_comm::ExtStatePtr & msg);

  /// this callback sends the full state (position, velocity) to the helicopter and bypasses the Luenberger Observer on the HLP
  /** the velocity has to be expressed in the same frame as the pose currently
   */
  void cbOdometry(const nav_msgs::OdometryConstPtr& msg);

  void sendPoseToAP(const double & x, const double & y, const double & z, const double & yaw,
                    const unsigned char & qual);

  /// sends parameters to HLP
  bool sendParameters(const asctec_hl_interface::SSDKConfig & config);
  void processDebugData(uint8_t * buf, uint32_t length);
  void processStatusData(uint8_t * buf, uint32_t length);
  void cbSSDKConfig(asctec_hl_interface::SSDKConfig & config, uint32_t level);

public:
  SSDKInterface(ros::NodeHandle & nh, CommPtr & comm);
  ~SSDKInterface();
};

#endif /* SSDK_INTERFACE_H_ */
