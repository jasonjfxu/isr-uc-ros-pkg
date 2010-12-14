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
* Author: Gonçalo Cabrita on 10/11/2010
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <lse_sensor_msgs/Anemometer.h>

ros::Publisher wind_pub;
std::string frame_id;

void newDataCallback(std::string * data)
{
	data->erase(data->size()-1, 1);
	data->erase(0, 1);

	//ROS_INFO("Received <STX>%s<ETX>", data->c_str());
	
	lse_sensor_msgs::Anemometer wind_msg;
	int direction;
	float speed;
	
	int first_comma = data->find_first_of(",", 0);
	int second_comma = data->find_first_of(",", first_comma+1);
	if(second_comma-first_comma==1)
	{
		sscanf(data->c_str(), "Q,,%f,M,00,", &speed);
		direction = 0;
	}
	else
	{
		sscanf(data->c_str(), "Q,%d,%f,M,00,", &direction, &speed);
		if(direction>180) direction -= 360;
	}
	
	wind_msg.header.stamp = ros::Time::now();
	wind_msg.header.frame_id = frame_id.c_str();
	
	wind_msg.min_wind_speed = 0.05;
	wind_msg.max_wind_speed = 60.00;
	
	wind_msg.wind_speed = speed;
	wind_msg.wind_direction = direction*0.0174532925;
	
	wind_pub.publish(wind_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "windsonic_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	wind_pub = n.advertise<lse_sensor_msgs::Anemometer>("/wind", 10);
	
	std::string port;
	int baudrate;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	pn.param("baudrate", baudrate, 38400);
	pn.param<std::string>("frame_id", frame_id, "/base_anemometer");

	ROS_INFO("%s", frame_id.c_str());
	
	cereal::CerealPort serial_port;

	try{ serial_port.open((char*)port.c_str(), baudrate); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("WindSonic -- Failed to open serial port!");
		ROS_BREAK();
	}
	ROS_INFO("WindoSonic -- Successfully connected to the WindSonic!");
	
	if( !serial_port.startReadBetweenStream(boost::bind(&newDataCallback, _1), 0x02, 0x03) )
	{
		ROS_FATAL("WindSonic -- Failed to start streaming data!");
		ROS_BREAK();
	}
	ROS_INFO("Windsonic -- Starting to stream data...");

	ros::spin();

  	return(0);
}

// EOF

