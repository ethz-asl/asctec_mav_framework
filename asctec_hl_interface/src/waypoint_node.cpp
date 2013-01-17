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
#include <asctec_hl_comm/WPControllerCommand.h>
#include <asctec_hl_comm/WPControllerWaypoint.h>
#include <asctec_hl_comm/WaypointActionAbort.h>
#include <dynamic_reconfigure/server.h>
#include <asctec_hl_interface/WaypointNodeConfig.h>

boost::mutex current_pose_mutex_;
std::vector<asctec_hl_comm::WaypointGoal> wp_list_;
ros::Publisher abort_wp_;
bool allow_wp_;
double pos_acc_, yaw_acc_, goal_yaw_, timeout_;

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

void wp_listCB(const asctec_hl_comm::WPControllerCommandConstPtr & msg)
{
	if (allow_wp_)
	{
		boost::mutex::scoped_lock lock(current_pose_mutex_);

		ROS_INFO_STREAM("Waypoint list received - parsing...");
		int n_tot=0;

		// abort current waypoint
		asctec_hl_comm::WaypointActionAbort abort_msg;
		abort_msg.cancel_id=1;
		abort_wp_.publish(abort_msg);

		// parse msg for new waypointlist
		n_tot = msg->pts.size();
		wp_list_.clear();
		for(int i=0;i<n_tot;++i)
		{
			wp_list_[i].header.seq=i;
			wp_list_[i].header.stamp = msg->header.stamp;

			wp_list_[i].goal_pos.x = msg->pts[i].pos.x;
			wp_list_[i].goal_pos.y = msg->pts[i].pos.y;
			wp_list_[i].goal_pos.z = msg->pts[i].pos.z;
			wp_list_[i].max_speed.x = msg->pts[i].speed;
			wp_list_[i].max_speed.y = msg->pts[i].speed;
			wp_list_[i].max_speed.z = msg->pts[i].speed;
			wp_list_[i].goal_yaw = goal_yaw_;
			wp_list_[i].accuracy_orientation = yaw_acc_;
			wp_list_[i].accuracy_position = pos_acc_;
			wp_list_[i].timeout = timeout_;
		}
		ROS_INFO_STREAM("...successfully parsed " << n_tot << " waypoints.");
	}
}


void cbConfig(asctec_hl_interface::WaypointNodeConfig & config, uint32_t level)
{
	pos_acc_ = config.pos_var;
	yaw_acc_ = config.yaw_var;
	goal_yaw_ = config.goal_yaw;
	timeout_ = config.timeout;

	if(config.cancel_wp)
	{
		allow_wp_=false;
	    boost::mutex::scoped_lock lock(current_pose_mutex_);
		wp_list_.clear();
		asctec_hl_comm::WaypointActionAbort abort_msg;
		abort_msg.cancel_id=1;
		abort_wp_.publish(abort_msg);
	}
	else
	{
		allow_wp_=true;
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_node");
	ros::NodeHandle pnh("waypoint_node");
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> ac(nh, "fcu/waypoint", true);
	asctec_hl_comm::WaypointGoal goal;
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started");

	// bring up dynamic reconfigure

	typedef dynamic_reconfigure::Server<asctec_hl_interface::WaypointNodeConfig> ReconfigureServer;
	ReconfigureServer reconf_srv_(pnh);
	ReconfigureServer::CallbackType f = boost::bind(&cbConfig, _1, _2);
	reconf_srv_.setCallback(f);

	abort_wp_ = nh.advertise<asctec_hl_comm::WaypointActionAbort> ("fcu/abort_action", 1);

	ros::Subscriber wp_list = pnh.subscribe("wp_list", 1, &wp_listCB);

	//  print published/subscribed topics
	ros::V_string topics;
	ros::this_node::getSubscribedTopics(topics);
	std::string nodeName = ros::this_node::getName();
	std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	topicsStr += "\tadvertised topics:\n";
	ros::this_node::getAdvertisedTopics(topics);
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	ROS_INFO_STREAM(""<< topicsStr);

	// work through waypointlist
	while(ros::ok())
	{
		if(allow_wp_ & (wp_list_.size()>0))
		{
			boost::mutex::scoped_lock lock(current_pose_mutex_);

			ac.sendGoal(wp_list_.front(), &doneCb, &activeCb, &feedbackCB);

			//wait for the action to return
			ros::Duration t(wp_list_.front().timeout);
			bool finished_before_timeout = ac.waitForResult(t);
			if (!finished_before_timeout)
				ROS_WARN_STREAM("Action did not finish before the time out. Resending waypoint...");
			else
				wp_list_.erase(wp_list_.begin());
		}
		ros::spinOnce();
	}


	return 0;
}
