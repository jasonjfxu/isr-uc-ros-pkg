/*
 *  plumesim_test.cpp
 *  
 *
 *  Created by Gonçalo Cabrita on 07/06/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	PlumeSim for ROS test node.
 *
 */


#include "ros/ros.h"
#include "plumesim/ReadPlumeSim.h"
#include <nav_msgs/Odometry.h>
#include <cstdlib>

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
  	ros::ServiceClient client = n.serviceClient<plumesim::ReadPlumeSim>("read_plumesim");

	ros::Rate r(1);
  	while(ros::ok())
  	{
		plumesim::ReadPlumeSim srv;
	  	srv.request.odom.pose.pose.position.x = myOdom.pose.pose.position.x;
		srv.request.odom.pose.pose.position.y = myOdom.pose.pose.position.y;
		srv.request.odom.pose.pose.position.z = myOdom.pose.pose.position.z;

	  	if(client.call(srv))
	  	{
	    		ROS_INFO("Chemical: %f", srv.response.sniff.reading[0]);
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
