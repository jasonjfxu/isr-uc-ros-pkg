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
* Author: Gonçalo Cabrita and Pedro Sousa on 07/06/2010
*********************************************************************/
#include "ros/ros.h"
#include "plumesim/ReadPlumeSim.h"
#include <nav_msgs/Odometry.h>
#include <cstdlib>

using namespace plumesim;

nav_msgs::Odometry myOdom;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	myOdom.pose.pose.position.x = msg->pose.pose.position.x;
	myOdom.pose.pose.position.y = msg->pose.pose.position.y;
	myOdom.pose.pose.position.z = msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "plumesim_test");

  	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);
  	ros::ServiceClient client = n.serviceClient<plumesim::ReadPlumeSim>("plumesim/read_plumesim");

	ros::Rate r(1);
  	while(ros::ok())
  	{
		plumesim::ReadPlumeSim srv;
	  	srv.request.odom.pose.pose.position.x = myOdom.pose.pose.position.x;
		srv.request.odom.pose.pose.position.y = myOdom.pose.pose.position.y;
		srv.request.odom.pose.pose.position.z = myOdom.pose.pose.position.z;

	  	if(client.call(srv))
	  	{
	    		ROS_INFO("Chemical: %f", srv.response.nostril.reading);
	  	}
	  	else
	  	{
	    		ROS_ERROR("Failed to get a reading from PlumeSim.");
	  	}
		ros::spinOnce();
		r.sleep();
	}

 	return(0);
}
