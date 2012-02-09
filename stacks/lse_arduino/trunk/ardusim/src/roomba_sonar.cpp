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
* Author: Gonçalo Cabrita on 18/10/2010
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ardusim/Ardusim.h"

using namespace ardusim;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "roomba_sonar_node");
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::vector<lse_sensor_msgs::Range> range_msgs;
	
	ros::Publisher range_pub = n.advertise<lse_sensor_msgs::Range>("/sonars", 1);
	
	// Read of type c-string/char buffer
	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB1");
	std::string frame_id;
	pn.param<std::string>("frame_id", frame_id, "/base_sonar");

	Ardusim ardusim(port);
	
	int requests[] = {ARDUSIM_RANGE};
	ardusim.setRequests(requests, 1);

	ros::Rate r(10);
  	while(ros::ok())
	{
		if(ardusim.getSensorData(100))
		{
			if(ardusim.getRange(&range_msgs))
			{
				range_msgs[0].header.frame_id = frame_id;
				range_msgs[0].header.frame_id += "_0";
				range_msgs[1].header.frame_id = frame_id;
				range_msgs[1].header.frame_id += "_1";
				range_msgs[2].header.frame_id = frame_id;
				range_msgs[2].header.frame_id += "_2";
				range_msgs[3].header.frame_id = frame_id;
				range_msgs[3].header.frame_id += "_3";
				range_msgs[4].header.frame_id = frame_id;
				range_msgs[4].header.frame_id += "_4";
			
				for(int i=0 ; i<range_msgs.size() ; i++)
				{
					range_msgs[i].field_of_view = 0.523598776;	// 30 degress
					range_msgs[i].min_range = 0.03;
					range_msgs[i].max_range = 2.00;
					range_pub.publish(range_msgs[i]);
				}
			}
		}
		
		r.sleep();
	}

  	return(0);
}

// EOF

