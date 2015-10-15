/*
 * gps_conversion.h
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
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h>
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
#include <geodetic_utils/geodetic_conv.hpp>

#include <memory>

namespace asctec_hl_gps{
class GpsConversion
{
 public:
  GpsConversion();

 private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
      asctec_hl_comm::mav_imu> GpsImuSyncPolicy;

  const Eigen::Quaterniond Q_90_DEG;

  void syncCallback(const sensor_msgs::NavSatFixConstPtr & gps,
                    const asctec_hl_comm::mav_imuConstPtr & imu);
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr & gps);
  void gpsCustomCallback(const asctec_hl_comm::GpsCustomConstPtr & gps);
  void imuCallback(const sensor_msgs::ImuConstPtr & imu);                       // For simulation
  void imuCustomCallback(const asctec_hl_comm::mav_imuConstPtr & imu);
  void filteredOdometryCallback(const nav_msgs::Odometry & filtered_odometry);

  bool zeroHeightCb(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & resp);
  bool wgs84ToEnuSrv(asctec_hl_comm::Wgs84ToEnuRequest & wgs84Pt,
                     asctec_hl_comm::Wgs84ToEnuResponse & enuPt);

  ros::NodeHandle nh_;
  ros::Publisher gps_pose_pub_;
  ros::Publisher gps_pose_nocov_pub_;
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
  ros::Subscriber imu_custom_sub_;
  ros::Subscriber filtered_odometry_sub_;

  geodetic_converter::GeodeticConverter geodetic_converter_;
  geometry_msgs::Point gps_position_;
  sensor_msgs::NavSatFix gps_message_;

  bool usePressureHeight_;
  bool setHeightZero_;

  bool isSim_;

  double height_offset_;

}; // class GpsConverter
}  // end asctec_hl_gps namespace

#endif /* GPS_FUSION_H_ */
