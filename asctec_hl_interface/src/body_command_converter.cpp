/*
 * body_command_converter.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: acmarkus
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <asctec_hl_comm/mav_ctrl.h>

class CommandConverter
{
private:
  ros::Subscriber cmd_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher cmd_pub_;

  geometry_msgs::PoseStamped current_pose_;

  float getYaw(const geometry_msgs::Quaternion & q)
  {
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  bool timeOk(const ros::Time & time)
  {
    bool ret = std::abs((current_pose_.header.stamp - time).toSec()) < 0.5;
    if (!ret)
      ROS_WARN_STREAM_THROTTLE(1, "Timestamps of control and current pose are not in sync("<<time<<", "<<current_pose_.header.stamp<<"), not publishing");
    return ret;
  }

  void sendVelocityBody(const asctec_hl_comm::mav_ctrlConstPtr & msg)
  {
    if (!timeOk(msg->header.stamp))
      return;

    float yaw = getYaw(current_pose_.pose.orientation);
    float s_yaw = sin(yaw);
    float c_yaw = cos(yaw);

    asctec_hl_comm::mav_ctrlPtr msg_out(new asctec_hl_comm::mav_ctrl);
    *msg_out = *msg;

    msg_out->type = asctec_hl_comm::mav_ctrl::velocity;
    msg_out->x = c_yaw * msg->x - s_yaw * msg->y;
    msg_out->y = s_yaw * msg->x + c_yaw * msg->y;

    cmd_pub_.publish(msg_out);
  }

  void sendPositionBody(const asctec_hl_comm::mav_ctrlConstPtr & msg)
  {
    if (!timeOk(msg->header.stamp))
      return;

    if(std::abs(msg->yaw) > M_PI){
      ROS_WARN("yaw has to be in [-pi ... pi], got %f instead", msg->yaw);
      return;
    }

    float yaw = getYaw(current_pose_.pose.orientation);
    float s_yaw = sin(yaw);
    float c_yaw = cos(yaw);

    asctec_hl_comm::mav_ctrlPtr msg_out;
    *msg_out = *msg;

    msg_out->type = asctec_hl_comm::mav_ctrl::position;
    msg_out->x = c_yaw * msg->x - s_yaw * msg->y;
    msg_out->y = s_yaw * msg->x + c_yaw * msg->y;

    yaw += msg->yaw;
    msg_out->yaw = yaw > M_PI ? (yaw - 2 * M_PI) : yaw;

    cmd_pub_.publish(msg_out);
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
  {
    current_pose_ = *msg;
  }

  void cmdCallback(const asctec_hl_comm::mav_ctrlConstPtr & msg)
  {
    switch (msg->type)
    {
      case asctec_hl_comm::mav_ctrl::velocity:
        cmd_pub_.publish(msg);
        break;
      case asctec_hl_comm::mav_ctrl::position:
        cmd_pub_.publish(msg);
        break;
      case asctec_hl_comm::mav_ctrl::velocity_body:
        sendVelocityBody(msg);
        break;
      case asctec_hl_comm::mav_ctrl::position_body:
        sendPositionBody(msg);
        break;
      default:
        ROS_WARN("didn't understand command type %d",msg->type);
    }
  }

public:
  CommandConverter()
  {
    ros::NodeHandle nh;

    cmd_sub_ = nh.subscribe("fcu/control_body", 10, &CommandConverter::cmdCallback, this);
    pose_sub_ = nh.subscribe("fcu/current_pose", 10, &CommandConverter::poseCallback, this);

    cmd_pub_ = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 10);
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "command_Converter");
  CommandConverter cc;
  ros::spin();

  return 0;
}
