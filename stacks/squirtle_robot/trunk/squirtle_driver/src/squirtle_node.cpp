/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 06/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <squirtle_msgs/Status.h>		// squirtle_status

#include <lse_sensor_msgs/Nostril.h>		// chemical_sensor

#include <dynamic_reconfigure/server.h>
#include <squirtle_driver/SquirtleConfig.h>

#include "Squirtle.h"

Squirtle robot = Squirtle();

double in_place_rotation_scale;
double speed_scale;
double rotation_scale;
double linear_actuator_rotation_scale;

void reconfigureCallback(squirtle_driver::SquirtleConfig &config, uint32_t level)
{ 
	ROS_INFO("Squirtle - %s - Got a request to reconfigure the kinematic model  %lf %lf %lf %lf", __FUNCTION__, 
            config.in_place_rotation_scale, 
            config.speed_scale, 
            config.rotation_scale, 
            config.linear_actuator_rotation_scale);

	in_place_rotation_scale = config.in_place_rotation_scale;
	speed_scale = config.speed_scale;
	rotation_scale = config.rotation_scale;
	linear_actuator_rotation_scale = config.linear_actuator_rotation_scale;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	int left_motor;
	int right_motor;
	int linear_actuator;
	
	// If we want to rotate in place
    	if(cmd_vel->linear.x == 0 && cmd_vel->angular.z != 0)
	{
		linear_actuator = 0;
        	left_motor = (cmd_vel->linear.x * speed_scale) - (cmd_vel->angular.z * in_place_rotation_scale);
        	right_motor = (cmd_vel->linear.x * speed_scale) + (cmd_vel->angular.z * in_place_rotation_scale);
	}
	// Otherwise
	else
	{
        	linear_actuator = cmd_vel->angular.z * linear_actuator_rotation_scale;
        	left_motor = (cmd_vel->linear.x * speed_scale) - (cmd_vel->angular.z * rotation_scale);
        	right_motor = (cmd_vel->linear.x * speed_scale) + (cmd_vel->angular.z * rotation_scale);
	}
	
	robot.drive(left_motor, right_motor, linear_actuator);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "squirtle_node");

	ROS_INFO("Squirtle for ROS v0.1");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
    	pn.param("baudrate", baudrate, 19200);
	
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, cmdVelReceived);
	ros::Publisher status_pub = n.advertise<squirtle_msgs::Status>("squirtle/status", 10);

    ros::Publisher chem_pub = n.advertise<lse_sensor_msgs::Nostril>("chemical_sensor", 10);

	dynamic_reconfigure::Server<squirtle_driver::SquirtleConfig> server;
  	dynamic_reconfigure::Server<squirtle_driver::SquirtleConfig>::CallbackType reconfigure_callback = boost::bind(&reconfigureCallback, _1, _2);
  	server.setCallback(reconfigure_callback);
    
    // First we open the port...
    if(!robot.openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("Squirtle -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
    ROS_INFO("Squirtle -- Successfully connected to Squirtle ASV!");
    
    // Next we reset the Arduino, start listening to the serial port and wait for
    // the Arduino to send the START command
    ROS_INFO("Squirtle -- Initializing the Squirtle ASV...");
    if(!robot.initialize())
    {
        ROS_FATAL("Squirtle -- Failed to initialize the Squirtle ASV!");
        ROS_BREAK();
    }
    ros::Rate r(10);
    ros::Time start_time = ros::Time::now();
    while(!robot.isReady())
    {
        if(ros::Time::now() - start_time > ros::Duration(5.0))
        {
            ROS_FATAL("Squirtle -- Failed to initialize the Squirtle ASV!");
            ROS_BREAK();
        }
        r.sleep();
    }
    ROS_INFO("Squirtle -- Initialization complete.");

    // And finally we tell the Arduino to start streaming data
    ROS_INFO("Squirtle -- Starting data stream...");
    if(!robot.startStreaming(100, 500, 2000))
    {
        ROS_FATAL("Squirtle -- Failed to start streaming!");
        ROS_BREAK();
    }
    ROS_INFO("Squirtle -- Streaming data.");

	while(n.ok())
	{
        squirtle_msgs::Status status_msg;
		status_msg.stamp = ros::Time::now();

		status_msg.battery_voltage = robot.getBatteryVoltage();
		status_msg.battery_current = robot.getBatteryCurrent();
		status_msg.battery_temperature = robot.getBatteryTemperature();
		status_msg.solar_panel_voltage = robot.getSolarPanelVoltage();
		status_msg.solar_panel_current = robot.getSolarPanelCurrent();
		status_msg.left_motor_current = robot.getLeftMotorCurrent();
		status_msg.right_motor_current = robot.getRightMotorCurrent();
        status_msg.motor_driver_temperature = robot.getMotorDriverTemperature();
			
		status_pub.publish(status_msg);

        lse_sensor_msgs::Nostril chem_msg;
        chem_msg.header.frame_id = "cyclops";
        chem_msg.header.stamp = ros::Time::now();

        chem_msg.reading = robot.getChemicalSensorReading();

        chem_pub.publish(chem_msg);

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
