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
* Author: Gonçalo Cabrita and Pedro Sousa on 07/03/2011
*********************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <string>
#include <list>

#include <iostream> 
#include <sys/ioctl.h> 
#include <fcntl.h> 
#include <linux/kd.h> 

#include "ros/ros.h"
#include "lse_sensor_msgs/Nostril.h"
#include "cereal_port/CerealPort.h"

// Global variables
cereal::CerealPort serial_port;

struct Nose
{
	lse_sensor_msgs::Nostril nostril;
	FILE * csv_file;
	std::list<int> readings;
};
	
std::vector<Nose> noses;
double temperature;
double humidity;

bool fill_buffer;
int buffer_size;

// Helper functions
void noseCallback(const lse_sensor_msgs::Nostril::ConstPtr& msg)
{
	bool nostril_found = false;
	std::vector<Nose>::iterator n;
	for(n=noses.begin() ; n!=noses.end() ; n++)
	{
		if(n->nostril.header.frame_id.compare(msg->header.frame_id)==0 && n->nostril.sensor_model.compare(msg->sensor_model)==0)
		{
			n->nostril.header.stamp = msg->header.stamp;
			n->nostril.reading = msg->reading;
			
			if(fill_buffer)
			{
				n->readings.push_front((int)msg->reading);
				if(n->readings.size() > buffer_size) n->readings.pop_back();
			}
		
			nostril_found = true;
			break;
		}
	}
	
	if(!nostril_found)
	{
		ROS_INFO("eNose Calibration - %s - Found a new nostril %s %s", __FUNCTION__, msg->header.frame_id.c_str(), msg->sensor_model.c_str());
	
		std::string file_name;
		
		file_name.append(msg->sensor_model);
		file_name.append("_");
		file_name.append(msg->header.frame_id);
		file_name.append(".csv");
		
		// Lets take all '/' and ' ' out!
		for(int i=0 ; i<file_name.size() ; i++)
		{
			if(file_name[i] == '/' || file_name[i] == ' ') file_name[i] = '_';
		}
		
		ROS_INFO("eNose Calibration - %s - Opening file %s...", __FUNCTION__, file_name.c_str());
		
		Nose new_nose;
		new_nose.csv_file = fopen(file_name.c_str(), "w");
		new_nose.nostril = *msg;
		
		noses.push_back(new_nose);
	}
}

void newDataCallback(std::string * data)
{
	sscanf(data->c_str(), "@,%lf,%lf,e", &temperature, &humidity);
	
	//ROS_INFO("eNose Calibration - %s - %s T:%lf H:%lf", __FUNCTION__, data->c_str(), temperature, humidity);
}

void fanOn()
{
	ROS_INFO("eNose Calibration - %s - Turning the fan on...", __FUNCTION__);
	serial_port.write("@1");
}

void fanOff()
{
	serial_port.write("@0");
	ROS_INFO("eNose Calibration - %s - Turned the fan off.", __FUNCTION__);
}

void closeFiles()
{
	if(noses.size()>0)
	{
		ROS_INFO("eNose Calibration - %s - Closing files...", __FUNCTION__);
	
		std::vector<Nose>::iterator n;
		for(n=noses.begin() ; n!=noses.end() ; n++)
		{
			fclose(n->csv_file);
		}
	}
}

void beep()
{
	printf("%c", 7);
}

// Main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "nose_calibration");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	ROS_INFO("eNose Calibration Node for ROS v0.1");
	
	// Parameters
	std::string port;
	int baudrate;
	
	double volume;
	double density;
	double molar_volume;
	double molar_mass;
	
	int num_noses;
	double fan_on_time;
	double fan_off_time;
	
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	pn.param("baudrate", baudrate, 9600);
	
	pn.param("box_volume", volume, 1.0);
	pn.param("gas_density", density, 1.0);
	pn.param("gas_molar_volume", molar_volume, 1.0);
	pn.param("gas_molar_mass", molar_mass, 1.0);
	
	pn.param("number_of_noses", num_noses, 1);
	pn.param("number_of_samples", buffer_size, 10);
	pn.param("fan_on_time", fan_on_time, 10.0);
	pn.param("fan_off_time", fan_off_time, 3.0);

	// Serial port
	try{ serial_port.open((char*)port.c_str(), baudrate); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("eNose Calibration -- Failed to open serial port!");
		ROS_BREAK();
	}
	ROS_INFO("eNose Calibration -- Successfully connected to the arduino!");
	
	if( !serial_port.startReadBetweenStream(boost::bind(&newDataCallback, _1), '@', 'e') )
	{
		ROS_FATAL("eNose Calibration -- Failed to start streaming data!");
		ROS_BREAK();
	}
	ROS_INFO("eNose Calibration -- Starting to stream data...");
	
	ros::Subscriber sub = n.subscribe("nose", 100, noseCallback);
	
	ros::Rate r(1.0);
	
	// Wait for all noses...
	ROS_INFO("eNose Calibration -- Waiting for all noses to come online...");
	while(n.ok() && noses.size()<num_noses)
	{
		ros::spinOnce();
		r.sleep();
	}
	
	std::vector<Nose>::iterator n_it;
	
	// Main loop
	double chemical_volume = 0.0;
	fill_buffer = false;	
	while(n.ok())
	{
		if(!fill_buffer)
		{
			char input[8];
			printf("[QUERY] How much chemical did you add? ");
			scanf("%s", input);
			
			if(input[0]=='q' || input[0]=='Q')
			{
				closeFiles();
				ROS_INFO("eNose Calibration -- Goodbye!");
				return 0;
			}
			
			chemical_volume += (double)atoi(input);
			
			fanOn();
			ros::Duration(fan_on_time).sleep();
			fanOff();
			ros::Duration(fan_off_time).sleep();
			beep();
			
			fill_buffer = true;
		}
		else
		{
			bool found_uncomplete_buffer = false;
			for(n_it=noses.begin() ; n_it!=noses.end() ; n_it++)
			{
				if(n_it->readings.size() < buffer_size)
				{
					found_uncomplete_buffer = true;
					break;
				}
			}
			
			if(!found_uncomplete_buffer)
			{
				for(n_it=noses.begin() ; n_it!=noses.end() ; n_it++)
				{
					n_it->readings.sort();
					while(n_it->readings.size() > buffer_size/2) n_it->readings.pop_front();
				
					double ppm = (molar_volume/molar_mass)*(density*chemical_volume*1000/volume);
				
					fprintf(n_it->csv_file, "%.6lf,%d,%.2lf,%.2lf\n", ppm, n_it->readings.front(), temperature, humidity);
					
					n_it->readings.clear();
				}
				
				fill_buffer = false;
			}
		}
	
		ros::spinOnce();
		r.sleep();
	}
	
	closeFiles();
	ROS_INFO("eNose Calibration -- Goodbye!");
	return 0;
}

// EOF
