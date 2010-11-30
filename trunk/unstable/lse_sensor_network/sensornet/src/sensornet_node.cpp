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
* Author: Gonçalo Cabrita and Pedro Sousa on 25/10/2010
*********************************************************************/
#define NODE_VERSION 0.01

#include "SensorNet.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sensornet_node");

	ROS_INFO("SensorNet for ROS %.2f", NODE_VERSION);
	
	ros::NodeHandle n;
	
	std::vector<lse_sensor_msgs::Nostril> odorMsgs;
	std::vector<lse_sensor_msgs::Nostril>::iterator odorIt;
	ros::Publisher odor_pub = n.advertise<lse_sensor_msgs::Nostril>("/sensornet_odor", 1200);
	
	std::string port;
	n.param<std::string>("sensornet/port", port, "/dev/ttyUSB0");
	
	SensorNet sn(port, 19200);
	std::vector<SensorNet::Node>::iterator node;
	
	ROS_INFO("SensorNet - Scanning nodes...");
	sn.scanNodes(1);
	if(sn.nodes.size()==0)
	{
		ROS_ERROR("SensorNet - Found 0 nodes! Quitting program...");
		return -1;
	}
	ROS_INFO("SensorNet - Found %d node%s", sn.nodes.size(), sn.nodes.size()==1 ? "!" : "s!");
	
	for(node=sn.nodes.begin() ; node!=sn.nodes.end() ; node++)
	{
		sn.getSensors(&(*node));
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO("SensorNet - Syncing nodes...");
	sn.syncNodes();
	
	ros::Duration(0.5).sleep();
	
	double delay = 3.0;
	ros::Time start = ros::Time::now();
	for(node=sn.nodes.begin() ; node!=sn.nodes.end() ; node++)
	{
		sn.setTime(&(*node), start + ros::Duration(delay));
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("SensorNet - Syncing complete, initiating data publish...");
	ros::Duration(delay+10).sleep();
	
	ros::Rate r(0.1);
	while(ros::ok())
	{
		odorMsgs.clear();
		for(node=sn.nodes.begin() ; node!=sn.nodes.end() ; node++)
		{
			sn.getSensorReadings(&(*node), &odorMsgs, NULL);
			ros::Duration(0.1).sleep();
		}
		
		int n=0;
		for(odorIt=odorMsgs.begin() ; odorIt!=odorMsgs.end() ; odorIt++)
		{
			odor_pub.publish(*odorIt);
			n++;
		}
			
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
