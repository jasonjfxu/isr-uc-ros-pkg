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
* Author: Gonçalo Cabrita on 15/10/2010
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "Ardusim.h"

using namespace ardusim;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nose_node");
	
	ros::NodeHandle n("~");
	
	std::vector<lse_sensor_msgs::Nostril> nose_msgs;
	
	ros::Publisher nose_pub = n.advertise<lse_sensor_msgs::Nostril>("/nose", 1);
	
	// Read of type c-string/char buffer
	std::string port;
	n.param<std::string>("port", port, "/dev/ttyUSB1");
	std::string frame_id;
	n.param<std::string>("frame_id", frame_id, "/base_nose");

	Ardusim ardusim(port);
	
	int requests[] = {ARDUSIM_NOSE};
	ardusim.setRequests(requests, 1);

	ros::Rate r(10);
  	while(ros::ok())
	{
		if(ardusim.getSensorData(100))
		{
			if(ardusim.getNose(&nose_msgs))
			{
				nose_msgs[0].sensor_model = "Figaro 2620";
				nose_msgs[0].gas_type.push_back(lse_sensor_msgs::Nostril::ORGANIC_SOLVENTS);
			
				nose_msgs[1].sensor_model = "Figaro 2600";
				nose_msgs[1].gas_type.push_back(lse_sensor_msgs::Nostril::AIR_CONTAMINANTS);
			
				for(int i=0 ; i<nose_msgs.size() ; i++)
				{
					nose_msgs[i].header.frame_id = frame_id;
					nose_msgs[i].min_reading = 0.0;
					nose_msgs[i].max_reading = 3300.0;
					nose_pub.publish(nose_msgs.at(i));
				}
			}
		}
		
		r.sleep();
	}

  	return(0);
}

// EOF

