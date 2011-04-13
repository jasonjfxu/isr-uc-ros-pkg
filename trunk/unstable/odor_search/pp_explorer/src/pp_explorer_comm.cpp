/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 05/03/2011
*********************************************************************/

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <lse_sensor_msgs/Nostril.h>
#include <pp_explorer/ForeignNostril.h>
#include "wifi_comm/wifi_comm_lib.h"

using namespace wifi_comm;

class PPExplorerComm
{
	public:
	PPExplorerComm();
	~PPExplorerComm();
	
	private:
	WiFiComm * comm;
	ros::NodeHandle n_;
	
	//! Nose topic subscriber.
	message_filters::Subscriber<lse_sensor_msgs::Nostril> nose_sub_;
	//! Transform listener.
	tf::TransformListener tf_;
	tf::MessageFilter<lse_sensor_msgs::Nostril> * tf_filter_;
	
	tf::TransformBroadcaster broadcaster_;
	
	ros::Publisher foreign_nose_pub_;
	ros::Subscriber foreign_nose_sub_;
	
	ros::Publisher nose_pub_;
	
	void localNoseCallback(const boost::shared_ptr<const lse_sensor_msgs::Nostril>& msg);
	void othersNoseCallback(const pp_explorer::ForeignNostril::ConstPtr& msg);
	void robotJoinedNetwork(char * ip);
};

PPExplorerComm::PPExplorerComm() : n_()
{
	comm = new WiFiComm(boost::bind(&PPExplorerComm::robotJoinedNetwork, this, _1));

	nose_sub_.subscribe(n_, "/nose", 10);
	tf_filter_ = new tf::MessageFilter<lse_sensor_msgs::Nostril>(nose_sub_, tf_, "/map", 50);
    tf_filter_->registerCallback( boost::bind(&PPExplorerComm::localNoseCallback, this, _1) );
    
    foreign_nose_pub_ = n_.advertise<pp_explorer::ForeignNostril>("/my_nose", 10);
 	foreign_nose_sub_ = n_.subscribe("/others_nose", 50, &PPExplorerComm::othersNoseCallback, this);
 	
 	nose_pub_ = n_.advertise<lse_sensor_msgs::Nostril>("/nose", 10);
}

PPExplorerComm::~PPExplorerComm()
{
	delete comm;
}

void PPExplorerComm::localNoseCallback(const boost::shared_ptr<const lse_sensor_msgs::Nostril>& msg)
{	
	if(msg->header.frame_id.compare("dummy") != 0) return;

	geometry_msgs::PoseStamped nose;
	nose.header.frame_id = msg->header.frame_id;
	nose.header.stamp = msg->header.stamp;
	nose.pose.position.x = 0.0;	
	nose.pose.position.y = 0.0;
	nose.pose.position.z = 0.0;
	
	geometry_msgs::PoseStamped pp_pose;
	try 
	{
		// Transform the bubble center from the chemical sensor frame to the map frame
		tf_.transformPose("/map", nose, pp_pose);
	}
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("PPExplorerComm -- Error: %s", ex.what());
		return;
	}
	
	pp_explorer::ForeignNostril my_nose_msg;
	my_nose_msg.nostril = *msg;
	my_nose_msg.pose = pp_pose;
	
	foreign_nose_pub_.publish(my_nose_msg);
}

void PPExplorerComm::othersNoseCallback(const pp_explorer::ForeignNostril::ConstPtr& msg)
{
	lse_sensor_msgs::Nostril nose_msg = msg->nostril;
	nose_msg.header.stamp = ros::Time::now();
	
	nose_pub_.publish(nose_msg);
	
	broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)), ros::Time::now(), msg->pose.header.frame_id.c_str()
	
	, msg->nostril.header.frame_id.c_str()));
}

void PPExplorerComm::robotJoinedNetwork(char * ip)
{
	// Send
	comm->openForeignRelay(ip, "/my_nose", "/others_nose");
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "pp_explorer_comm");

	PPExplorerComm myComm;

	ros::spin();

	return 0;
}

// EOF

