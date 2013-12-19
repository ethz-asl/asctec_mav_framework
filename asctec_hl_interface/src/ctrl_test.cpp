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

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>

void usage()
{
  std::string text("usage: \n\n");
  text = "ctrl_test motors [0 | 1]\n";
  text += "ctrl_test ctrl [acc | vel | pos | vel_b | pos_b] x y z yaw\n";
  text += "position / velocity in [m] / [m/s] and yaw in [deg] / [deg/s] (-180 ... 180)\n";
  std::cout << text << std::endl;
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "interfacetest");
  ros::NodeHandle nh;

  ros::Publisher pub;

  if (argc == 1)
  {
    ROS_ERROR("Wrong number of arguments!!!");
    usage();
    return -1;
  }

  std::string command = std::string(argv[1]);

  if (command == "motors")
  {
    if (argc != 3)
    {
      ROS_ERROR("Wrong number of arguments!!!");
      usage();
      return -1;
    }

    asctec_hl_comm::mav_ctrl_motors::Request req;
    asctec_hl_comm::mav_ctrl_motors::Response res;
    req.startMotors = atoi(argv[2]);
    ros::service::call("fcu/motor_control", req, res);
    std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
  }
  else if (command == "ctrl")
  {
    if (argc != 7)
    {
      ROS_ERROR("Wrong number of arguments!");
      usage();
      return -1;
    }
    asctec_hl_comm::mav_ctrl msg;
    msg.x = atof(argv[3]);
    msg.y = atof(argv[4]);
    msg.z = atof(argv[5]);
    msg.yaw = atof(argv[6]) * M_PI / 180.0;
    msg.v_max_xy = -1; // use max velocity from config
    msg.v_max_z = -1;

    std::string type(argv[2]);
    if (type == "acc")
      msg.type = asctec_hl_comm::mav_ctrl::acceleration;
    else if (type == "vel")
      msg.type = asctec_hl_comm::mav_ctrl::velocity;
    else if (type == "pos")
      msg.type = asctec_hl_comm::mav_ctrl::position;
    else if (type == "vel_b")
      msg.type = asctec_hl_comm::mav_ctrl::velocity_body;
    else if (type == "pos_b")
      msg.type = asctec_hl_comm::mav_ctrl::position_body;
    else
    {
      ROS_ERROR("Command type not recognized");
      usage();
      return -1;
    }

    pub = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
    ros::Rate r(15); // ~15 Hz

    for (int i = 0; i < 15; i++)
    {
      pub.publish(msg);
      if (!ros::ok())
        return 0;
      r.sleep();
    }

    // reset
    if (type != "pos" && type != "pos_b")
    {
      msg.x = 0;
      msg.y = 0;
      msg.z = 0;
      msg.yaw = 0;
    }

    for(int i=0; i<5; i++){
      pub.publish(msg);
      r.sleep();
    }

    ros::spinOnce();
  }

  return 0;
}
