/*
 * gps_fusion_node.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: acmarkus
 */

#include "gps_conversion.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "gps_conversion");

  asctec_hl_gps::GpsConversion gf;

  ros::spin();

  return 0;
}
