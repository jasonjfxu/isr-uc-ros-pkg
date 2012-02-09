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
* Author: Gonçalo Cabrita and Pedro Sousa on 17/11/2010
*********************************************************************/
#include <vector>
#include <string>

#include <ros/ros.h>
#include <fctuc_open_day/Poses.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_reader_node");

	if(argc!=2)
	{
		ROS_ERROR("Wrong number of arguments %d!", argc);
		ROS_ERROR("rosrun fctuc_open_day pose_reader filename");
		return 0;
	}
	
	ros::NodeHandle n;
	
	ros::Publisher pub  = n.advertise<fctuc_open_day::Poses>("/poses", 10);
	
	ROS_INFO("Opening file %s...", argv[1]);
	
	FILE * goalsFile;
	goalsFile = fopen(argv[1], "r");
	if(goalsFile == NULL)
	{
		ROS_FATAL("Cannot open file %s", argv[3]);
		return 0;	
	}
	
	fctuc_open_day::Poses targets;
	targets.header.frame_id = "/map";
	targets.header.stamp = ros::Time::now();
	
	geometry_msgs::Pose2D pose;
	// Might come in handy... some day :P
	float z, qx, qy, qz, qw;
	while(!feof(goalsFile))
	{
		fscanf(goalsFile, "Frame:/map, Position(%lf, %lf, %f), Orientation(%f, %f, %f, %f) = Angle: %lf\n", &(pose.x), &(pose.y), &z, &qx, &qy, &qz, &qw, &(pose.theta));
		targets.poses.push_back(pose);
		
		ROS_INFO("Frame:/map, Position(%lf, %lf, %f), Orientation(%f, %f, %f, %f) = Angle: %f", pose.x, pose.y, z, qx, qy, qz, qw, pose.theta);
	}
	
	ROS_INFO("Publishing poses...");
	for(int i=0 ; i<5 ; i++)
	{
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	pub.publish(targets);

	ROS_INFO("Done!");

	return 0;
}
