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

#include "ardusim/Ardusim.h"

using namespace ardusim;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nose_node");
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::vector<lse_sensor_msgs::Nostril> nose_msgs;
	
	ros::Publisher nose_pub = n.advertise<lse_sensor_msgs::Nostril>("/nose", 1);
	
	// Read of type c-string/char buffer
	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB1");
	std::string frame_id;
	pn.param<std::string>("frame_id", frame_id, "/base_nose");
	
	double clean_air_2620;
	pn.param("clean_air_2620", clean_air_2620, 0.0);
	double clean_air_2600;
	pn.param("clean_air_2600", clean_air_2600, 0.0);
	
	double min_2620;
	pn.param("min_2620", min_2620, 0.0);
	double max_2620;
	pn.param("max_2620", max_2620, 0.0);
	
	double min_2600;
	pn.param("min_2600", min_2600, 1000.0);
	double max_2600;
	pn.param("max_2600", max_2600, 1000.0);
	
	double a_2620;
	pn.param("a_2620", a_2620, 1.0);
	double b_2620;
	pn.param("b_2620", b_2620, 1.0);
	double a_2600;
	pn.param("a_2600", a_2600, 1.0);
	double b_2600;
	pn.param("b_2600", b_2600, 1.0);

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
				nose_msgs[0].clean_air = clean_air_2620;
				nose_msgs[0].reading = exp((nose_msgs[0].raw_data-b_2620)/a_2620);
				nose_msgs[0].min_reading = min_2620;
				nose_msgs[0].max_reading = max_2620;
			
				nose_msgs[1].sensor_model = "Figaro 2600";
				nose_msgs[1].gas_type.push_back(lse_sensor_msgs::Nostril::AIR_CONTAMINANTS);
				nose_msgs[1].clean_air = clean_air_2600;
				nose_msgs[1].reading = exp((nose_msgs[1].raw_data-b_2600)/a_2600);
				nose_msgs[1].min_reading = min_2600;
				nose_msgs[1].max_reading = max_2600;
			
				for(int i=0 ; i<nose_msgs.size() ; i++)
				{
					nose_msgs[i].header.frame_id = frame_id;
					nose_pub.publish(nose_msgs.at(i));
				}
			}
		}
		
		r.sleep();
	}

  	return(0);
}

// EOF

