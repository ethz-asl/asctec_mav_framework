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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <asctec_hl_comm/WaypointAction.h>

void feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr & fb)
{
  const geometry_msgs::Point32 & wp = fb->current_pos;
  ROS_INFO("got feedback: %fm %fm %fm %f° ", wp.x, wp.y, wp.z, fb->current_yaw*180/M_PI);
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result)
{
  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  const geometry_msgs::Point32 & wp = result->result_pos;
  ROS_INFO("Reached waypoint: %fm %fm %fm %f°",wp.x, wp.y, wp.z, result->result_yaw*180/M_PI);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "waypoint_client");
  ros::NodeHandle nh;

  float maxSpeed;
  float accuracy;
  ros::Duration timeout(15);

  if (argc == 5)
  {
    maxSpeed = 0.5;
    accuracy = 0.3;
  }
  else if (argc == 6)
  {
    maxSpeed = atof(argv[5]);
    accuracy = 0.3;
  }
  else if (argc == 7)
  {
    maxSpeed = atof(argv[5]);
    accuracy = atof(argv[6]);
  }
  else if (argc == 8)
  {
    maxSpeed = atof(argv[5]);
    accuracy = atof(argv[6]);
    timeout = ros::Duration(atof(argv[7]));
  }
  else
  {
    std::cout << "Wrong number of arguments! \nusage:" << "\twpclient x y z yaw \n"
        << "\twpclient x y z yaw max_speed\n" << "\twpclient x y z yaw max_speed accuracy\n"
        << "\twpclient x y z yaw max_speed accuracy timeout\n" << std::endl;
    return -1;
  }

  actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> ac(nh, "fcu/waypoint", true);

  asctec_hl_comm::WaypointGoal goal;

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");

  goal.goal_pos.x = atof(argv[1]);
  goal.goal_pos.y = atof(argv[2]);
  goal.goal_pos.z = atof(argv[3]);

  goal.max_speed.x = maxSpeed;
  goal.max_speed.y = maxSpeed;
  goal.max_speed.z = maxSpeed;

  goal.goal_yaw = atof(argv[4]);

  goal.accuracy_position = accuracy;
  goal.accuracy_orientation = 0;

  goal.timeout = timeout.toSec();

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCB);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(timeout);

  if (!finished_before_timeout)
  {
    ROS_WARN("Action did not finish before the time out.");
  }

  return 0;
}
