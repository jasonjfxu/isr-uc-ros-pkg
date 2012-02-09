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

void mapFrame(std::string * name, const char * base_name, int id)
{
	char temp_name[128];
	sprintf(temp_name, "%s_%d", base_name, id);

	name->clear();
	name->assign(temp_name);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "general_node");
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::vector<lse_sensor_msgs::Range> range_msgs;
	std::vector<lse_sensor_msgs::Nostril> nose_msgs;
	std::vector<lse_sensor_msgs::TPA> tpa_msgs;
	std::vector<lse_sensor_msgs::Anemometer> anemometer_msgs;
	
	ros::Publisher range_pub;
	ros::Publisher nose_pub;
	ros::Publisher tpa_pub;
	ros::Publisher anemometer_pub;
	
	// Read of type c-string/char buffer
	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	std::string sonar_frame_id;
	pn.param<std::string>("sonar_frame_id", sonar_frame_id, "/base_sonar");
	std::string nose_frame_id;
	pn.param<std::string>("nose_frame_id", nose_frame_id, "/base_nose");
	std::string tpa_frame_id;
	pn.param<std::string>("tpa_frame_id", tpa_frame_id, "/base_tpa");
	std::string anemometer_frame_id;
	pn.param<std::string>("anemometer_frame_id", anemometer_frame_id, "/base_anemometer");
	std::string file_path;
	pn.param<std::string>("calibration_file_path", file_path, "anemometer.csv");
	bool scan_sensors;
	pn.param("scan_sensors", scan_sensors, true);

	Ardusim ardusim(port);
	
	int counter=0;
	if(scan_sensors)
	{
		while(!ardusim.sensorDiscovery(500))
		{
			counter++;
			if(counter==3)
			{
				ROS_FATAL("Ardusim GPnode - Failed to scan sensors for request from the Arduino!");
				ROS_BREAK();
			}
			ROS_WARN("Ardusim GPnode - Retrying to aquire sensors for request from the Arduino...");
		}
	}
	else
	{
		while(!ardusim.setAutoRequests(500))
		{
			counter++;
			if(counter==3)
			{
				ROS_FATAL("Ardusim GPnode - Failed to aquire sensors for request from the Arduino!");
				ROS_BREAK();
			}
			ROS_WARN("Ardusim GPnode - Retrying to aquire sensors for request from the Arduino...");
		}
	}
	
	std::list<int> requests;
	std::list<int>::iterator it;
	ardusim.getRequests(&requests);
	
	it=requests.begin();
	while(it!=requests.end())
	{
		if(*it==ARDUSIM_RANGE)
		{
			range_pub = n.advertise<lse_sensor_msgs::Range>("range", 1);
			ROS_INFO("Ardusim GPnode - Advertising lse_sensor_msgs::Range on topic /range");
		}
		else if(*it==ARDUSIM_NOSE)
		{
			nose_pub = n.advertise<lse_sensor_msgs::Nostril>("nose", 1);
			ROS_INFO("Ardusim GPnode - Advertising lse_sensor_msgs::Nostril on topic /nose");
		}
		else if(*it==ARDUSIM_TPA)
		{
			tpa_pub = n.advertise<lse_sensor_msgs::TPA>("tpa", 1);
			ROS_INFO("Ardusim GPnode - Advertising lse_sensor_msgs::TPA on topic /tpa");
		}
		else if(*it==ARDUSIM_ANEMOMETER)
		{
			if(!ardusim.loadAnemometerCalibFile(&file_path))
			{
				ROS_FATAL("Ardusim GPnode - Could not load the anemometer calibration file!");
				ROS_BREAK();
			}
		
			anemometer_pub = n.advertise<lse_sensor_msgs::Anemometer>("wind", 1);
			ROS_INFO("Ardusim GPnode - Advertising lse_sensor_msgs::Anemometer on topic /anemometer");
		}
		else
		{
			ROS_WARN("Ardusim GPnode - Ops! Found an unknown sensor id %d", *it);
		}
		++it;
	}
	
	ROS_INFO("Ardusim GPnode - Initiating sensor data publishing...");

	int i;
	std::string frame_id;
	
	ros::Rate r(10);
  	while(ros::ok())
	{
		if(ardusim.getSensorData(100))
		{
			if(ardusim.getRange(&range_msgs))
			{
				ROS_INFO("Got Range!!!");
				for(i=0 ; i<range_msgs.size() ; i++)
				{
					if(range_msgs.size()==1) range_msgs[i].header.frame_id = sonar_frame_id;
					else
					{
						mapFrame(&frame_id, sonar_frame_id.c_str(), i);
						range_msgs[i].header.frame_id = frame_id;
					}
					range_msgs[i].field_of_view = 0.523598776;	// 30 degress
					range_msgs[i].min_range = 0.03;
					range_msgs[i].max_range = 2.00;
					
					range_pub.publish(range_msgs.at(i));
				}
			}
			if(ardusim.getNose(&nose_msgs))
			{
				for(i=0 ; i<nose_msgs.size() ; i++)
				{n.param<std::string>("anemometer_frame_id", anemometer_frame_id, "/base_anemometer");
					if(nose_msgs.size()==1) nose_msgs[i].header.frame_id = nose_frame_id;
					else
					{
						mapFrame(&frame_id, nose_frame_id.c_str(), i);
						nose_msgs[i].header.frame_id = frame_id;
					}
					nose_msgs[i].min_reading = 0.0;
					nose_msgs[i].max_reading = 3300.0;
					
					nose_pub.publish(nose_msgs.at(i));
				}
			}
			if(ardusim.getTPA(&tpa_msgs))
			{
				for(i=0 ; i<tpa_msgs.size() ; i++)
				{
					if(tpa_msgs.size()==1) tpa_msgs[i].header.frame_id = tpa_frame_id;
					else
					{
						mapFrame(&frame_id, tpa_frame_id.c_str(), i);
						tpa_msgs[i].header.frame_id = frame_id;
					}
					
					tpa_pub.publish(tpa_msgs.at(i));
				}
			}
			if(ardusim.getAnemometer(&anemometer_msgs))
			{
				for(i=0 ; i<anemometer_msgs.size() ; i++)
				{
					if(anemometer_msgs.size()==1) anemometer_msgs[i].header.frame_id = anemometer_frame_id;
					else
					{
						mapFrame(&frame_id, anemometer_frame_id.c_str(), i);
						anemometer_msgs[i].header.frame_id = frame_id;
					}
					
					anemometer_pub.publish(anemometer_msgs.at(i));
				}
			}
		}
		r.sleep();
	}
  	return(0);
}

// EOF

