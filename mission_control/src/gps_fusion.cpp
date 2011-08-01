/*
 * gps_fusion.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: acmarkus
 */

#include "gps_fusion.h"

void latlon2xy_dbg(double lat0, double lon0, double lat, double lon, double *X, double *Y) //X: East, Y: North in mm; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement
{
  const double MEAN_EARTH_DIAMETER = 12756274.0;
  const double UMR = 0.017453292519943295769236907684886; //PI/180

  *Y = sin((lat - lat0) / 2 * UMR) * MEAN_EARTH_DIAMETER;
  *X = sin((-lon + lon0) / 2 * UMR) * MEAN_EARTH_DIAMETER * cos(lat0 * UMR);
}

GpsFusion::GpsFusion() :
  nh_(""), gps_sub_sync_(nh_, "fcu/gps", 1), imu_sub_sync_(nh_, "fcu/imu_custom", 1), gps_imu_sync_(GpsImuSyncPolicy(10),
                                                                                          gps_sub_sync_, imu_sub_sync_),
      have_reference_(false), set_height_zero_(false),Q_M90_DEG(sqrt(2.0)/2.0, 0, 0, -sqrt(2.0)/2.0)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

//  gps_imu_sync_.registerCallback(boost::bind(&GpsFusion::syncCallback, this, _1, _2));
  //  gps_imu_sync_.setInterMessageLowerBound(0, ros::Duration(0.180)); // gps arrives at max with 5 Hz

  imu_sub_ = nh.subscribe("fcu/imu_custom", 1, &GpsFusion::imuCallback, this);
  gps_sub_ = nh.subscribe("fcu/gps", 1, &GpsFusion::gpsCallback, this);

  gps_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("gps_metric", 1);
  zero_height_srv_ = nh.advertiseService("set_height_zero", &GpsFusion::zeroHeightCb, this);

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


bool GpsFusion::zeroHeightCb(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & resp)
{
  set_height_zero_ = true;
  return true;
}

void GpsFusion::syncCallback(const sensor_msgs::NavSatFixConstPtr & gps, const asctec_hl_comm::mav_imuConstPtr & imu)
{
  if (gps->status.status != sensor_msgs::NavSatStatus::STATUS_FIX)
  {
    //    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    ROS_WARN_STREAM("No GPS fix");
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

    msg->header = gps->header;
    msg->pose.pose.orientation = imu->orientation; // assuming that magnetic compass is connected, then yaw is absolute
    msg->pose.pose.position = wgs84ToEnu(gps->latitude, gps->longitude, gps->altitude);
//    msg->pose.pose.position = wgs84ToNwu(gps->latitude, gps->longitude, gps->altitude);
    msg->pose.pose.position.z = imu->height - height_offset_;

    gps_pose_pub_.publish(msg);
  }
}

void GpsFusion::gpsCallback(const sensor_msgs::NavSatFixConstPtr & gps)
{
  if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
    gps_pose_ = wgs84ToEnu(gps->latitude, gps->longitude, gps->altitude);
  else{
    gps_pose_.x = gps_pose_.y = gps_pose_.z = 0;
  }
}

void GpsFusion::imuCallback(const asctec_hl_comm::mav_imuConstPtr & imu){
  if (gps_pose_.x == 0 && gps_pose_.y == 0 && gps_pose_.z == 0)
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
    orientation = Q_M90_DEG * orientation;
    msg->header = imu->header;
    msg->pose.pose.orientation.w = orientation.w();
    msg->pose.pose.orientation.x = orientation.x();
    msg->pose.pose.orientation.y = orientation.y();
    msg->pose.pose.orientation.z = orientation.z();
    msg->pose.pose.position = gps_pose_;
    msg->pose.pose.position.z = imu->height - height_offset_;

    gps_pose_pub_.publish(msg);
  }
}

void GpsFusion::initReference(const double & latitude, const double & longitude, const double & altitude)
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

Eigen::Vector3d GpsFusion::wgs84ToEcef(const double & latitude, const double & longitude, const double & altitude)
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

Eigen::Vector3d GpsFusion::ecefToEnu(const Eigen::Vector3d & ecef)
{
  return ecef_ref_orientation_ * (ecef - ecef_ref_point_);
}

geometry_msgs::Point GpsFusion::wgs84ToEnu(const double & latitude, const double & longitude, const double & altitude)
{
  geometry_msgs::Point ret;
  Eigen::Vector3d tmp;
  tmp = ecefToEnu(wgs84ToEcef(latitude, longitude, altitude));
  ret.x = tmp[0];
  ret.y = tmp[1];
  ret.z = tmp[2];

  // debug ...
  double dbg_x, dbg_y;
  latlon2xy_dbg(ref_latitude_, ref_longitude_, latitude, longitude, &dbg_x, &dbg_y);

  ROS_INFO("local metric coordinates: x: %f/%f y: %f/%f", ret.x, dbg_x, ret.y, dbg_y);

  return ret;
}

geometry_msgs::Point GpsFusion::wgs84ToNwu(const double & latitude, const double & longitude, const double & altitude)
{
  geometry_msgs::Point ret;
  Eigen::Vector3d tmp;
  tmp = ecefToEnu(wgs84ToEcef(latitude, longitude, altitude));
  ret.x = tmp[1];
  ret.y = -tmp[0];
  ret.z = tmp[2];

  // debug ...
  double dbg_x, dbg_y;
  latlon2xy_dbg(ref_latitude_, ref_longitude_, latitude, longitude, &dbg_x, &dbg_y);

  ROS_INFO("local metric coordinates: x: %f/%f y: %f/%f", ret.x, dbg_y, ret.y, -dbg_x);

  return ret;
}

