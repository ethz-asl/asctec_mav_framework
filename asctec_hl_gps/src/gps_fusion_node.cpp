/*
 * gps_fusion_node.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: acmarkus
 */

#include "gps_fusion.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "gps_fusion");

  GpsFusion gf;

  ros::spin();

  return 0;
}
