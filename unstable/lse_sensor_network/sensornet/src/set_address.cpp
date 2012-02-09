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
	ros::init(argc, argv, "set_address");
	ros::NodeHandle n;

	if(argc<2)
	{
		ROS_ERROR("Too few arguments. Usage: set_address new_addr old_addr='X'");
		return -1;
	}
	
	char new_address = argv[1][0];
	if(new_address<'A' || new_address>'X')
	{
		ROS_ERROR("The new address must be between A and X");
		return -1;
	}

	char old_address = 'Y';
	if(argc==3)
	{
		old_address = argv[2][0];
		if(old_address<'A' || old_address>'Y')
		{
			ROS_ERROR("The old address must be between A and Y");
			return -1;
		}
	}

	std::string port;
	n.param<std::string>("sensornet/port", port, "/dev/ttyUSB0");
	
	SensorNet sn(port, 19200);
	SensorNet::Node node;
	node.setAddress(old_address);
	
	if(!sn.setAddress(&node, new_address))
	{
		ROS_ERROR("There was an error while changing the address of node %c!", old_address);
	}
	else
	{
		ROS_INFO("Successfuly changed the address of node %c to %c!", old_address, new_address);
	}
	
	return 0;
}

// EOF
