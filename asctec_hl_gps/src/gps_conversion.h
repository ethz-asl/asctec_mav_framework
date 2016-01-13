/*
 * gps_fusion.h
 *
 *  Created on: Jun 22, 2011
 *      Author: acmarkus
 */

#ifndef GPS_CONVERSION_H_
#define GPS_CONVERSION_H_

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <asctec_hl_comm/PositionWithCovarianceStamped.h>
#include <asctec_hl_comm/mav_imu.h>
#include <asctec_hl_comm/GpsCustom.h>
#include <asctec_hl_comm/GpsCustomCartesian.h>
#include <asctec_hl_comm/Wgs84ToEnu.h>
#include <std_srvs/Empty.h>
#include <Eigen/Eigen>

#include "geodesy_ned.hpp"

#include <memory>

namespace asctec_hl_gps{

class GpsConversion
{
public:
  GpsConversion();

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, asctec_hl_comm::mav_imu> GpsImuSyncPolicy;

  static const double DEG2RAD;
  const Eigen::Quaterniond Q_90_DEG;

  ros::NodeHandle nh_;
  ros::Publisher gps_pose_pub_;
  ros::Publisher gps_position_pub_;
  ros::Publisher gps_position_nocov_pub_;
  ros::Publisher gps_custom_pub_;
  ros::Publisher gps_filtered_pub_;

  ros::ServiceServer zero_height_srv_;
  ros::ServiceServer gps_to_enu_srv_;

  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_sync_;
  message_filters::Subscriber<asctec_hl_comm::mav_imu> imu_sub_sync_;
  message_filters::Synchronizer<GpsImuSyncPolicy> gps_imu_sync_;

  ros::Subscriber gps_sub_;
  ros::Subscriber gps_custom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber filtered_odometry_sub_;

  geometry_msgs::Point gps_position_;

  bool have_reference_;
  double ref_latitude_;
  double ref_longitude_;
  double ref_altitude_;
  Eigen::Vector3d ecef_ref_point_;
  Eigen::Quaterniond ecef_ref_orientation_;

  double height_offset_;
  bool set_height_zero_;

  bool use_pressure_height_;

  std::unique_ptr<geodesy_ned::Ned> ned_;

  void syncCallback(const sensor_msgs::NavSatFixConstPtr & gps, const asctec_hl_comm::mav_imuConstPtr & imu);
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr & gps);
  void gpsCustomCallback(const asctec_hl_comm::GpsCustomConstPtr & gps);
  void imuCallback(const asctec_hl_comm::mav_imuConstPtr & imu);
  void filteredOdometryCallback(const nav_msgs::Odometry & filtered_odometry);

  void initReference(const double & latitude, const double & longitude, const double & altitude);

  Eigen::Vector3d wgs84ToEcef(const double & latitude, const double & longitude, const double & altitude);
  Eigen::Vector3d ecefToEnu(const Eigen::Vector3d & ecef);

  geometry_msgs::Point wgs84ToEnu(const double & latitude, const double & longitude, const double & altitude);
  geometry_msgs::Point wgs84ToNwu(const double & latitude, const double & longitude, const double & altitude);

  bool zeroHeightCb(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & resp);
  bool wgs84ToEnuSrv(asctec_hl_comm::Wgs84ToEnuRequest & wgs84Pt,
                     asctec_hl_comm::Wgs84ToEnuResponse & enuPt);
};

} // end namespace

#endif /* GPS_FUSION_H_ */
