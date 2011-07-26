/*
 * height_vicon_fusion.cpp
 *
 *  Created on: Jul 15, 2011
 *      Author: acmarkus
 */


#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <asctec_hl_comm/mav_imu.h>

geometry_msgs::Pose g_pose;
ros::Publisher g_pose_pub;

void imuCb(const asctec_hl_comm::mav_imuConstPtr & msg){
  geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped);

  pose->header = msg->header;
  pose->pose.pose = g_pose;
  pose->pose.pose.position.z = msg->height;

  g_pose_pub.publish(pose);
}

void vCb(const geometry_msgs::TransformStampedConstPtr & msg){
  g_pose.position.x = msg->transform.translation.x;
  g_pose.position.y = msg->transform.translation.y;
  g_pose.position.z = msg->transform.translation.z;

  g_pose.orientation = msg->transform.rotation;
}


int main(int argc, char** argv){

  ros::init(argc, argv, "vicon_height_fusion");
  ros::NodeHandle nh;

  ros::Subscriber sub_imu, sub_vicon;

  g_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("hex/fcu/pose", 1);
  sub_imu = nh.subscribe("hex/fcu/imu_custom", 1, imuCb);
  sub_vicon = nh.subscribe("vicon/sfly_hex/sfly_hex", 1, vCb);

  ros::spin();
}
