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

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

// action server
#include <actionlib/server/simple_action_server.h>
#include <asctec_hl_comm/WaypointAction.h>

typedef actionlib::SimpleActionServer<asctec_hl_comm::WaypointAction> WaypointActionServer;

class WPServer
{
private:
  ros::NodeHandle nh_;
  ros::Publisher wp_pub_;
  ros::Subscriber pose_sub_;

  WaypointActionServer *wp_server_;

  geometry_msgs::PoseStamped current_pose_;
  boost::mutex current_pose_mutex_;


  /// gets called on pose updates to determine if the current waypoint is reached
  void poseCallback(const geometry_msgs::PoseStampedConstPtr & pose)
  {
    boost::mutex::scoped_lock lock(current_pose_mutex_);
    current_pose_ = *pose;
  }

  /// accepts new goal waypoint and sends the helicopter there
  void wpExecuteCB(const asctec_hl_comm::WaypointGoalConstPtr & goal)
  {
    const geometry_msgs::Point32 & wp = goal->goal_pos;
    const float & yaw = goal->goal_yaw;

    ROS_INFO("got new waypoint: x=%f y=%f z=%f yaw=%f v_xy=%f v_z=%f accuracy=%f timeout=%f", wp.x, wp.y, wp.z, goal->goal_yaw, goal->max_speed.x, goal->max_speed.z, goal->accuracy_position, goal->timeout);

    asctec_hl_comm::mav_ctrlPtr new_wp(new asctec_hl_comm::mav_ctrl);
    new_wp->x = wp.x;
    new_wp->y = wp.y;
    new_wp->z = wp.z;
    new_wp->yaw = yaw;
    new_wp->v_max_xy = goal->max_speed.x;
    new_wp->v_max_z = goal->max_speed.z;
    new_wp->type = asctec_hl_comm::mav_ctrl::position;
    wp_pub_.publish(new_wp);

    std::stringstream feedback_msg;

    ros::Rate r(2);
    bool success = true;
    static asctec_hl_comm::WaypointResult result;
    static asctec_hl_comm::WaypointFeedback feedback;

    float dist_to_wp = 1e9;
    float dist_yaw = M_PI;
    float dx, dy, dz;

    int i = 0;
    int max_cycles = ceil(goal->timeout / r.expectedCycleTime().toSec());

    while (ros::ok() && i < max_cycles)
    {
      if (wp_server_->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("waypoint server: Preempted");
        // set the action state to preempted
        wp_server_->setPreempted();
        success = false;
        break;
      }

      boost::mutex::scoped_lock lock(current_pose_mutex_);

      dx = wp.x - current_pose_.pose.position.x;
      dy = wp.y - current_pose_.pose.position.y;
      dz = wp.z - current_pose_.pose.position.z;
      dist_to_wp = sqrt(dx * dx + dy * dy + dz * dz);

      dist_yaw = fabs(yaw - tf::getYaw(current_pose_.pose.orientation));
      dist_yaw = dist_yaw > M_PI ? fabs(dist_yaw - M_PI) : dist_yaw;

      feedback_msg.clear();
      feedback_msg.str("");
      feedback_msg << "dist: " << dist_to_wp << " dist_yaw" << dist_yaw;
      if (dist_to_wp <= goal->accuracy_position && dist_yaw <= goal->accuracy_orientation)
        break;

      feedback.current_pos.x = current_pose_.pose.position.x;
      feedback.current_pos.y = current_pose_.pose.position.y;
      feedback.current_pos.z = current_pose_.pose.position.z;
      feedback.current_yaw = tf::getYaw(current_pose_.pose.orientation);
      feedback.status = feedback_msg.str();

      lock.unlock();

      wp_server_->publishFeedback(feedback);

      r.sleep();
      i++;
    }

    result.result_pos.x = current_pose_.pose.position.x;
    result.result_pos.y = current_pose_.pose.position.y;
    result.result_pos.z = current_pose_.pose.position.z;
    result.result_yaw = tf::getYaw(current_pose_.pose.orientation);

    if (i == max_cycles)
    {
      success = false;
      wp_server_->setAborted(result, "Timed Out!");
    }

    if (success)
    {
      result.status = "Waypoint reached!";
      wp_server_->setSucceeded(result);
    }
  }

public:
  WPServer() :
    nh_(""), wp_server_(NULL)
  {
    wp_pub_ = nh_.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
    pose_sub_ = nh_.subscribe("fcu/current_pose", 10, &WPServer::poseCallback, this);
    wp_server_ = new WaypointActionServer(nh_, "fcu/waypoint", boost::bind(&WPServer::wpExecuteCB, this, _1), false);
    wp_server_->start();
  }
  ~WPServer()
  {
    wp_server_->shutdown();
    delete wp_server_;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_server");
  WPServer wps;
  ROS_INFO("Waypoint server started ... waiting for requests");
  ros::spin();
  return 0;
}
