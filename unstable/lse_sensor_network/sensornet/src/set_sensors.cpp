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
* Author: Gonçalo Cabrita and Pedro Sousa on 28/10/2010
*********************************************************************/
#define NODE_VERSION 0.01

#include <stdlib.h>
#include <stdio.h>
#include "SensorNet.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "set_sensors");
	ros::NodeHandle n;

	if(argc!=8)
	{
		ROS_ERROR("Wrong number of arguments. Usage: set_sensors addr s1 s2 s3 s4 w1 w1");
		return -1;
	}

	std::string port;
	n.param<std::string>("sensornet/port", port, "/dev/ttyUSB0");
	
	SensorNet sn(port, 19200);
	SensorNet::Node node;
	node.setAddress(argv[1][0]);
	
	if(!sn.getInfo(&node))
	{
		ROS_ERROR("Could not find node %c. Are you sure you have the right address? Is the node connected?", node.address());
		return -1;
	}
	
	if(!sn.setSensors(&node, atoi(argv[2])==1 ? 1 : 0, atoi(argv[3])==1 ? 1 : 0, atoi(argv[4])==1 ? 1 : 0, atoi(argv[5])==1 ? 1 : 0, atoi(argv[6])==1 ? 1 : 0, atoi(argv[7])==1 ? 1 : 0))
	{
		ROS_ERROR("There was an error while setting the sensors on node %c!", node.address());
	}
	else
	{
		ROS_INFO("Successfuly changed the sensors on node %c!", node.address());
	}
	
	return 0;
}

// EOF
