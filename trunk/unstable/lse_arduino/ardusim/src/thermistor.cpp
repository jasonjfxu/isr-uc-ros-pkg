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
* Author: Gonçalo Cabrita on 04/11/2010
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ardusim/Ardusim.h"

using namespace ardusim;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "thermistor_node");
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::vector<lse_sensor_msgs::Thermistor> raw_msgs;
	
	ros::Publisher raw_pub = n.advertise<lse_sensor_msgs::Thermistor>("/thermistors", 10);
	
	// Read of type c-string/char buffer
	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	std::string frame_id;
	pn.param<std::string>("frame_id", frame_id, "/base_thermistor");

	Ardusim ardusim(port);
	
	int requests[] = {ARDUSIM_ANEMOMETER};
	ardusim.setRequests(requests, 1);

	ros::Rate r(10);
  	while(ros::ok())
	{
		if(ardusim.getSensorData(100))
		{
			if(ardusim.getThermistor(&raw_msgs))
			{
				for(int i=0 ; i<raw_msgs.size() ; i++)
				{
					char numbered_frame_id[128];
					raw_msgs[i].header.frame_id = frame_id;
					sprintf(numbered_frame_id, "_%d", i);
					raw_msgs[i].header.frame_id += numbered_frame_id;
					raw_pub.publish(raw_msgs.at(i));
				}
			}
		}
		r.sleep();
	}
  	return(0);
}

// EOF

