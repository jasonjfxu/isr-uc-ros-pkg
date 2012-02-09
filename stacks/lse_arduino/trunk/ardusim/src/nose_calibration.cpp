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
#include <sys/types.h>
#include <unistd.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "lse_sensor_msgs/Nostril.h"
#include "cereal_port/CerealPort.h"
#include "std_msgs/UInt16.h"

class NoseCalibration
{
	public:
	NoseCalibration();
	~NoseCalibration();
	
	void userInterface();
	void mainLoop();
	
	private:
	
	// Serial Port
	cereal::CerealPort serial_port;
	
	ros::NodeHandle n;
	ros::NodeHandle pn;
	
	ros::Subscriber sub;
	
	struct Nose
	{
		lse_sensor_msgs::Nostril nostril;
		FILE * csv_file;
		std::list<int> readings;
		ros::Publisher pub;
	};
	std::vector<Nose> noses;
	
	double temperature;
	double humidity;
	
	double chemical_volume;
	
	bool fill_buffer;
	
	// Parameters
	std::string port;
	int baudrate;
	
	double volume;
	double density;
	double molar_volume;
	double molar_mass;
	
	int num_noses;
	int buffer_size;
	
	double fan_on_time;
	double fan_off_time;
	
	// Functions
	void noseCallback(const lse_sensor_msgs::Nostril::ConstPtr& msg);
	void arduinoCallback(std::string * data);
	
	void fanOn();
	void fanOff();
	void beep();
	void closeFiles();
};

NoseCalibration::NoseCalibration() : n(), pn("~"), serial_port()
{
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
	
	if( !serial_port.startReadBetweenStream(boost::bind(&NoseCalibration::arduinoCallback, this, _1), '@', 'e') )
	{
		ROS_FATAL("eNose Calibration -- Failed to start streaming data!");
		ROS_BREAK();
	}
	ROS_INFO("eNose Calibration -- Starting to stream data...");
	
	fill_buffer = false;
	sub = n.subscribe("nose", 100, &NoseCalibration::noseCallback, this);
	
	chemical_volume = 0.0;
	
	// Wait for all noses...
	ROS_INFO("eNose Calibration -- Waiting for all noses to come online...");
	ros::Rate r(1.0);
	while(n.ok() && noses.size()<num_noses)
	{
		ros::spinOnce();
		r.sleep();
	}
	
	ROS_INFO("eNose Calibration -- Got all noses!");
}

NoseCalibration::~NoseCalibration()
{
	serial_port.close();
}

void NoseCalibration::userInterface()
{ 
	ROS_INFO("eNose Calibration - %s - User Interface", __FUNCTION__);

	ros::Rate r(1.0);
	char input[8];
		
	while(n.ok())
	{
		if(!fill_buffer)
		{
			printf("[QUERY] How much chemical did you add? ");
			scanf("%s", input);
			getchar();
		
			if(input[0]=='q' || input[0]=='Q')
			{
				closeFiles();
				ROS_INFO("eNose Calibration - %s - Press CTRL+C to exit.", __FUNCTION__);
				return;
			}
		
			chemical_volume += (double)atoi(input);
		
			fanOn();
		
			if(input[0]=='a' || input[0]=='A')
			{
				ros::Duration(fan_on_time).sleep();
			}
			else
			{
				printf("[QUERY] Press ENTER to stop the fan... ");
				fflush(stdout);
				getchar();
			}
		
			fanOff();
			ros::Duration(fan_off_time).sleep();
			beep();
		
			fill_buffer = true;
		}
		r.sleep();
	}
}

void NoseCalibration::mainLoop()
{
	ROS_INFO("eNose Calibration - %s - Main Loop", __FUNCTION__);

	ros::Rate r(1.0);
	std::vector<Nose>::iterator n_it;
		
	while(n.ok())
	{
		if(fill_buffer)
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
				fill_buffer = false;
				double ppm;
			
				for(n_it=noses.begin() ; n_it!=noses.end() ; n_it++)
				{
					n_it->readings.sort();
					while(n_it->readings.size() > buffer_size/2) n_it->readings.pop_front();
			
					ppm = (molar_volume/molar_mass)*(density*chemical_volume*1000/volume);
								
					fprintf(n_it->csv_file, "%.6lf,%d,%.2lf,%.2lf\n", ppm, n_it->readings.front(), temperature, humidity);
				
					n_it->readings.clear();
				}
				
				ROS_INFO("eNose Calibration - %s - Wrote to file. Now at %duL %.3lfppm", __FUNCTION__, (int)chemical_volume, ppm);
			}
		}
		
		for(n_it=noses.begin() ; n_it!=noses.end() ; n_it++)
		{
			std_msgs::UInt16 view_nose_msg;
			view_nose_msg.data = n_it->nostril.raw_data;
			n_it->pub.publish(view_nose_msg);
		}
		
		ros::spinOnce();
		r.sleep();
	}
}

void NoseCalibration::noseCallback(const lse_sensor_msgs::Nostril::ConstPtr& msg)
{
	bool nostril_found = false;
	std::vector<Nose>::iterator n_it;
	for(n_it=noses.begin() ; n_it!=noses.end() ; n_it++)
	{
		if(n_it->nostril.header.frame_id.compare(msg->header.frame_id)==0 && n_it->nostril.sensor_model.compare(msg->sensor_model)==0)
		{
			n_it->nostril.header.stamp = msg->header.stamp;
			n_it->nostril.raw_data = msg->raw_data;
			
			if(fill_buffer)
			{
				n_it->readings.push_front((int)msg->raw_data);
				if(n_it->readings.size() > buffer_size) n_it->readings.pop_back();
			}
			
			nostril_found = true;
			break;
		}
	}
	
	if(!nostril_found)
	{
		ROS_INFO("eNose Calibration - %s - Found a new nostril %s %s", __FUNCTION__, msg->header.frame_id.c_str(), msg->sensor_model.c_str());
	
		Nose new_nose;
	
		std::string file_name;
		
		file_name.append(msg->sensor_model);
		//file_name.append("_");
		file_name.append(msg->header.frame_id);
		
		// Lets take all '/' and ' ' out!
		for(int i=0 ; i<file_name.size() ; i++)
		{
			if(file_name[i] == '/' || file_name[i] == ' ') file_name[i] = '_';
		}
		
		ROS_INFO("eNose Calibration - %s - Publishing raw data on topic /%s...", __FUNCTION__, file_name.c_str());
		
		new_nose.pub = n.advertise<std_msgs::UInt16>(file_name, 10);
		
		ROS_INFO("eNose Calibration - %s - Opening file %s...", __FUNCTION__, file_name.c_str());
		
		file_name.append(".csv");
		new_nose.csv_file = fopen(file_name.c_str(), "w");
		new_nose.nostril = *msg;
		
		noses.push_back(new_nose);
	}
}

void NoseCalibration::arduinoCallback(std::string * data)
{
	sscanf(data->c_str(), "@,%lf,%lf,e", &temperature, &humidity);
	
	//ROS_INFO("eNose Calibration - %s - %s T:%lf H:%lf", __FUNCTION__, data->c_str(), temperature, humidity);
}

void NoseCalibration::fanOn()
{
	ROS_INFO("eNose Calibration - %s - Turning the fan on...", __FUNCTION__);
	serial_port.write("@1");
}

void NoseCalibration::fanOff()
{
	serial_port.write("@0");
	ROS_INFO("eNose Calibration - %s - Turned the fan off.", __FUNCTION__);
}

void NoseCalibration::beep()
{
	serial_port.write("@b");
	printf("%c", 7);
}

void NoseCalibration::closeFiles()
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

// Main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "nose_calibration");
	
	ROS_INFO("eNose Calibration Node for ROS v0.1");
	
	NoseCalibration nc;
	
	boost::thread t(boost::bind(&NoseCalibration::userInterface, &nc));
	
	nc.mainLoop();
	
	t.join();
	
	ROS_INFO("eNose Calibration -- Goodbye!");
	return 0;
}

// EOF
