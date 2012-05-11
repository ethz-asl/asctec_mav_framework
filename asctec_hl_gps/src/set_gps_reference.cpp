/*
 * set_gps_reference.cpp
 *
 *  Created on: Jun 29, 2011
 *      Author: acmarkus
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_its;


void gpsCb(const sensor_msgs::NavSatFixConstPtr & msg){

  static int count = 1;

  if (msg->status.status != sensor_msgs::NavSatStatus::STATUS_FIX)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    return;
  }

  g_lat_ref += msg->latitude;
  g_lon_ref += msg->longitude;
  g_alt_ref += msg->altitude;

  ROS_INFO("Current measurement: %f %f %f", msg->latitude, msg->longitude, msg->altitude);

  if(count == g_its){
    g_lat_ref /= g_its;
    g_lon_ref /= g_its;
    g_alt_ref /= g_its;

    ros::NodeHandle nh;
    nh.setParam("/gps_ref_latitude", g_lat_ref);
    nh.setParam("/gps_ref_longitude", g_lon_ref);
    nh.setParam("/gps_ref_altitude", g_alt_ref);

    ROS_INFO("final reference position: %f %f %f", g_lat_ref, g_lon_ref, g_alt_ref);

    ros::shutdown();
    return;
  }

  count ++;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "set_gps_reference");
  ros::NodeHandle nh;

  g_lat_ref = 0;
  g_lon_ref = 0;
  g_alt_ref = 0;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if(args.size() > 1)
    g_its = atoi(args[1].c_str());
  else
    g_its = 50;

  ROS_INFO("taking %d measurements\n",g_its);

  ros::Subscriber gps_sub = nh.subscribe("gps", 1, &gpsCb);

  ros::spin();
}
