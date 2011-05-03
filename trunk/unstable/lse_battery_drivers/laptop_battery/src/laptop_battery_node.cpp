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
* Author: Gonçalo Cabrita on 01/03/2011
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>
#include <laptop_battery/Battery.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "laptop_battery_node");
	ros::NodeHandle n;
	
	ROS_INFO("LaptopBattery for ROS v0.1");
	
	ros::Publisher pub = n.advertise<laptop_battery::Battery>("/laptop/battery", 10);
	
	char description[128];
	char value[128];
	
	int full_capacity;
	int remaining_capacity;
	int present_rate;
	
	laptop_battery::Battery bat_msg;
	
	ros::Rate r(1.0);
	while(n.ok())
	{	
		FILE * state = fopen("/proc/acpi/battery/BAT0/state", "r");
		FILE * info = fopen("/proc/acpi/battery/BAT0/info", "r");
	
		if(state && info)
		{
			while(!feof(state))
			{
				fscanf(state, "%[^:]:%[^\n]\n", description, value);
			
				//ROS_INFO("%s %s", description, value);
			
				if(strcmp(description, "remaining capacity")==0)
				{
					remaining_capacity = atoi(value);
				}
				else if(strcmp(description, "present rate")==0)
				{
					present_rate = atoi(value);
				}
				else if(strcmp(description, "charging state")==0)
				{
					std::string charge = value;
					size_t i = charge.find_last_of(' ');
					
					if(charge.compare(i+1, charge.length()-i+1, "discharging")==0) bat_msg.charging = false;
					else bat_msg.charging = true;
				}
			}
			
			while(!feof(info))
			{
				fscanf(info, "%[^:]:%[^\n]\n", description, value);
				
				//ROS_INFO("%s %s", description, value);
			
				if(strcmp(description, "last full capacity")==0)
				{
					full_capacity = atoi(value);
				}
			}
			
			if(full_capacity > 0) bat_msg.level = (float)(remaining_capacity*100.0/full_capacity);
			else ROS_ERROR("LaptopBattery -- Error calculating the battery level!");
			
			if(present_rate <= 0 || bat_msg.charging == true) bat_msg.time_remaining = 0;
			else bat_msg.time_remaining = (int)(remaining_capacity/present_rate*60);
			
			bat_msg.header.stamp = ros::Time::now();
			
			pub.publish(bat_msg);
		}
		else
		{
			ROS_FATAL("LaptopBattery -- Failed to read the battery info!");
			ROS_BREAK();
		}
		
		fclose(state);
		fclose(info);
	
		ros::spinOnce();
		r.sleep();
		
		//rewind(state);
		//rewind(info);
	}

  	return(0);
}

// EOF

