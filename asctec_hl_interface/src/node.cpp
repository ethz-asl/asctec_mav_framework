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

#include <boost/shared_ptr.hpp>
#include "comm.h"
#include "hl_interface.h"
#include "ssdk_interface.h"
#include "ekf_interface.h"

void printTopicInfo()
{
  //  print published/subscribed topics
  std::string nodeName = ros::this_node::getName();
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  ROS_INFO_STREAM(""<< topicsStr);
}

class Watchdog{
private:
  CommPtr & comm_;
  const double PERIOD;
  ros::Timer timer;
  uint32_t last_rx_count_;
  uint32_t timeout_count_;
  void check(const ros::TimerEvent & e)
  {
    if(last_rx_count_ == comm_->getRxPacketsGood()){
      timeout_count_++;
      ROS_WARN("No new valid packets within the last %f s",PERIOD);
    }

    if(timeout_count_ > 4){
      // a bit harsh, but: roslaunch will restart if "respawn" is set true, and no idea what would happen with all the static local variables :(( --> TODO: remove those!
      ROS_FATAL("No valid packets within the last %f s, aborting !", 5*PERIOD);
      ros::Duration(0.1).sleep(); //sleep a bit such that the above message will still get transmitted
      ros::shutdown();
    }

    last_rx_count_ = comm_->getRxPacketsGood();
  }

public:
  Watchdog(CommPtr & comm):comm_(comm), PERIOD(1), last_rx_count_(0), timeout_count_(0)
  {
    ROS_INFO("creating watchdog monitoring successful rx packets");
    ros::NodeHandle nh;
    timer = nh.createTimer(ros::Duration(PERIOD), &Watchdog::check, this);
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fcu");
  ros::NodeHandle nh("fcu");


  std::string port, portRX, portTX;
  int baudrate;
  ros::NodeHandle pnh("~/fcu");
  CommPtr comm(new Comm);

  bool connected = false;

  pnh.param("baudrate", baudrate, HLI_DEFAULT_BAUDRATE);

  if (pnh.getParam("serial_port_rx", portRX) && pnh.getParam("serial_port_tx", portTX))
  {
    connected = comm->connect(portRX, portTX, baudrate);
  }
  else
  {
    pnh.param("serial_port", port, std::string("/dev/ttyUSB0"));
    connected = comm->connect(port, port, baudrate);
  }

  if (!connected)
  {
    ROS_ERROR("unable to connect");
    ros::Duration(2).sleep();
    return -1;
  }

  HLInterface asctecInterface(nh, comm);
  SSDKInterface ssdk_if(nh, comm);
  EKFInterface ekf_if(nh, comm);

  printTopicInfo();

  Watchdog wd(comm);

  ros::spin();


  return 0;
}
