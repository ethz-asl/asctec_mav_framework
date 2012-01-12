/*
 * gps_fusion.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: acmarkus
 */

#include "gps_conversion.h"

namespace asctec_hl_gps{

GpsConversion::GpsConversion() :
  nh_(""), gps_sub_sync_(nh_, "fcu/gps", 1), imu_sub_sync_(nh_, "fcu/imu_custom", 1), gps_imu_sync_(GpsImuSyncPolicy(10),
                                                                                          gps_sub_sync_, imu_sub_sync_),
      have_reference_(false), set_height_zero_(false),Q_90_DEG(sqrt(2.0)/2.0, 0, 0, sqrt(2.0)/2.0)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  imu_sub_ = nh.subscribe("fcu/imu_custom", 1, &GpsConversion::imuCallback, this);
  gps_sub_ = nh.subscribe("fcu/gps", 1, &GpsConversion::gpsCallback, this);
  gps_custom_sub_ = nh.subscribe("fcu/gps_custom", 1, &GpsConversion::gpsCustomCallback, this);

  gps_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("fcu/gps_pose", 1);
  gps_position_pub_ = nh_.advertise<asctec_hl_comm::PositionWithCovarianceStamped> ("fcu/gps_position", 1);
  gps_custom_pub_ = nh_.advertise<asctec_hl_comm::GpsCustomCartesian> ("fcu/gps_position_custom", 1);
  zero_height_srv_ = nh.advertiseService("set_height_zero", &GpsConversion::zeroHeightCb, this);

  pnh.param("use_pressure_height", use_pressure_height_, false);

  if(use_pressure_height_)
  {
    gps_imu_sync_.registerCallback(boost::bind(&GpsConversion::syncCallback, this, _1, _2));
    gps_imu_sync_.setInterMessageLowerBound(0, ros::Duration(0.180)); // gps arrives at max with 5 Hz
  }

  if (nh.getParam("/gps_ref_latitude", ref_latitude_))
  {
    if (nh.getParam("/gps_ref_longitude", ref_longitude_))
    {
      if (nh.getParam("/gps_ref_altitude", ref_altitude_))
      {
        initReference(ref_latitude_, ref_longitude_, ref_altitude_);
        have_reference_ = true;
      }
    }
  }
}


bool GpsConversion::zeroHeightCb(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & resp)
{
  set_height_zero_ = true;
  return true;
}

void GpsConversion::syncCallback(const sensor_msgs::NavSatFixConstPtr & gps, const asctec_hl_comm::mav_imuConstPtr & imu)
{
  if (gps->status.status != sensor_msgs::NavSatStatus::STATUS_FIX)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    return;
  }

  if (!have_reference_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  if(set_height_zero_){
    set_height_zero_ = false;
    height_offset_ = imu->height;
  }

  if (use_pressure_height_)
  {
    asctec_hl_comm::PositionWithCovarianceStampedPtr msg(new asctec_hl_comm::PositionWithCovarianceStamped);
    msg->header = gps->header;
    msg->position = wgs84ToEnu(gps->latitude, gps->longitude, gps->altitude);
    msg->position.z = imu->height - height_offset_;

    gps_position_pub_.publish(msg);
  }
}

void GpsConversion::gpsCallback(const sensor_msgs::NavSatFixConstPtr & gps)
{
  if (!have_reference_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
    gps_position_ = wgs84ToEnu(gps->latitude, gps->longitude, gps->altitude);
    if (!use_pressure_height_)
    {
      asctec_hl_comm::PositionWithCovarianceStampedPtr msg(new asctec_hl_comm::PositionWithCovarianceStamped);
      msg->header = gps->header;
      msg->position = gps_position_;
      msg->covariance = gps->position_covariance;
      gps_position_pub_.publish(msg);
    }
  }
  else{
    gps_position_.x = gps_position_.y = gps_position_.z = 0;
  }
}

void GpsConversion::gpsCustomCallback(const asctec_hl_comm::GpsCustomConstPtr & gps)
{
  if (!have_reference_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
    geometry_msgs::Point pos = wgs84ToEnu(gps->latitude, gps->longitude, gps->altitude);

      asctec_hl_comm::GpsCustomCartesianPtr msg(new asctec_hl_comm::GpsCustomCartesian);
      msg->header = gps->header;
      msg->position = pos;
      msg->position_covariance = gps->position_covariance;
      msg->velocity_x = gps->velocity_x;
      msg->velocity_y = gps->velocity_y;
      msg->velocity_covariance = gps->velocity_covariance;

      if (use_pressure_height_)
      {
        msg->position.z = gps->pressure_height - height_offset_;
      }
      gps_custom_pub_.publish(msg);
  }

}

void GpsConversion::imuCallback(const asctec_hl_comm::mav_imuConstPtr & imu){
  if (gps_position_.x == 0 && gps_position_.y == 0 && gps_position_.z == 0)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    return;
  }

  if (!have_reference_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  if(set_height_zero_){
    set_height_zero_ = false;
    height_offset_ = imu->height;
  }

  if (gps_pose_pub_.getNumSubscribers() > 0)
  {
    geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);

    // magnetic compass is zero when pointing north, need to rotate measurement 90 deg towards east to be consistent with ENU

    Eigen::Quaterniond orientation(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    orientation = Q_90_DEG * orientation;
    msg->header = imu->header;
    msg->pose.pose.orientation.w = orientation.w();
    msg->pose.pose.orientation.x = orientation.x();
    msg->pose.pose.orientation.y = orientation.y();
    msg->pose.pose.orientation.z = orientation.z();
    msg->pose.pose.position = gps_position_;
    msg->pose.pose.position.z = imu->height - height_offset_;

    gps_pose_pub_.publish(msg);
  }
}

void GpsConversion::initReference(const double & latitude, const double & longitude, const double & altitude)
{
  Eigen::Matrix3d R;
  double s_long, s_lat, c_long, c_lat;
  sincos(latitude * DEG2RAD, &s_lat, &c_lat);
  sincos(longitude * DEG2RAD, &s_long, &c_long);

  R(0, 0) = -s_long;
  R(0, 1) = c_long;
  R(0, 2) = 0;

  R(1, 0) = -s_lat * c_long;
  R(1, 1) = -s_lat * s_long;
  R(1, 2) = c_lat;

  R(2, 0) = c_lat * c_long;
  R(2, 1) = c_lat * s_long;
  R(2, 2) = s_lat;

  ecef_ref_orientation_ = Eigen::Quaterniond(R);

  ecef_ref_point_ = wgs84ToEcef(latitude, longitude, altitude);
}

Eigen::Vector3d GpsConversion::wgs84ToEcef(const double & latitude, const double & longitude, const double & altitude)
{
  const double a = 6378137.0; // semi-major axis
  const double e_sq = 6.69437999014e-3; // first eccentricity squared

  double s_long, s_lat, c_long, c_lat;
  sincos(latitude * DEG2RAD, &s_lat, &c_lat);
  sincos(longitude * DEG2RAD, &s_long, &c_long);

  const double N = a / sqrt(1 - e_sq * s_lat * s_lat);

  Eigen::Vector3d ecef;

  ecef[0] = (N + altitude) * c_lat * c_long;
  ecef[1] = (N + altitude) * c_lat * s_long;
  ecef[2] = (N * (1 - e_sq) + altitude) * s_lat;

  return ecef;
}

Eigen::Vector3d GpsConversion::ecefToEnu(const Eigen::Vector3d & ecef)
{
  return ecef_ref_orientation_ * (ecef - ecef_ref_point_);
}

geometry_msgs::Point GpsConversion::wgs84ToEnu(const double & latitude, const double & longitude, const double & altitude)
{
  geometry_msgs::Point ret;
  Eigen::Vector3d tmp;
  tmp = ecefToEnu(wgs84ToEcef(latitude, longitude, altitude));
  ret.x = tmp[0];
  ret.y = tmp[1];
  ret.z = tmp[2];

  // debug ...
//  double dbg_x, dbg_y;
//  latlon2xy_dbg(ref_latitude_, ref_longitude_, latitude, longitude, &dbg_x, &dbg_y);
//  ROS_INFO("local metric coordinates: x: %f/%f y: %f/%f", ret.x, dbg_x, ret.y, dbg_y);

  return ret;
}

geometry_msgs::Point GpsConversion::wgs84ToNwu(const double & latitude, const double & longitude, const double & altitude)
{
  geometry_msgs::Point ret;
  Eigen::Vector3d tmp;
  tmp = ecefToEnu(wgs84ToEcef(latitude, longitude, altitude));
  ret.x = tmp[1];
  ret.y = -tmp[0];
  ret.z = tmp[2];

  // debug ...
//  double dbg_x, dbg_y;
//  latlon2xy_dbg(ref_latitude_, ref_longitude_, latitude, longitude, &dbg_x, &dbg_y);
//  ROS_INFO("local metric coordinates: x: %f/%f y: %f/%f", ret.x, dbg_y, ret.y, -dbg_x);

  return ret;
}

} // end namespace

