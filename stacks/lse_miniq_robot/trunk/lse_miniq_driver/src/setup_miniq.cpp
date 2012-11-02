/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 11/09/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include "miniQ.h"

miniQ robot = miniQ();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "miniq_node");

	ROS_INFO("miniQ setup for ROS");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
	pn.param("baudrate", baudrate, 57600);

	// Setup parameters
	int starting_l_kp;
	pn.param("starting_l_kp", starting_l_kp, 350);
	int starting_l_ki;
	pn.param("starting_l_ki", starting_l_ki, 80);
	int starting_l_kd;
	pn.param("starting_l_kd", starting_l_kd, 0);

	int running_l_kp;
	pn.param("running_l_kp", running_l_kp, 150);
	int running_l_ki;
	pn.param("running_l_ki", running_l_ki, 20);
	int running_l_kd;
	pn.param("running_l_kd", running_l_kd, 2);

	int starting_r_kp;
	pn.param("starting_r_kp", starting_r_kp, 350);
	int starting_r_ki;
	pn.param("starting_r_ki", starting_r_ki, 80);
	int starting_r_kd;
	pn.param("starting_r_kd", starting_r_kd, 0);

	int running_r_kp;
	pn.param("running_r_kp", running_r_kp, 150);
	int running_r_ki;
	pn.param("running_r_ki", running_r_ki, 20);
	int running_r_kd;
	pn.param("running_r_kd", running_r_kd, 2);

	int battery_type;
	pn.param("battery_type", battery_type, 0);

	int mode;
	pn.param("mode", mode, 0);

	double odometry_d;
	pn.param("odometry_d", odometry_d, 0.0);
	double odometry_yaw;
	pn.param("odometry_yaw", odometry_yaw, 0.0);

	double gas_a;
	pn.param("gas_a", gas_a, 0.0);
	double gas_b;
	pn.param("gas_b", gas_b, 0.0);
    
    double timeout;
	pn.param("timeout", timeout, 0.0);
    
	if(!miniQ::openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Successfully connected to the miniQ!");
    
    	ros::Duration(2.5).sleep();
    
    	if(!robot.checkVersion())
	{
		ROS_FATAL("miniQ -- The firmware version of the miniQ robot is not compatible with this ROS node!");
		ROS_BREAK();
	}
    
	ROS_INFO("miniQ -- Setting communication mode to %s...", mode==0 ? "Serial" : "XBee API");
	if(!robot.setMode(mode)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();

	ROS_INFO("miniQ -- Setting battery type to %s...", battery_type==0 ? "Li-Ion" : "4 AA");
	if(!robot.setBatteryType(battery_type)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();
	
	ROS_INFO("miniQ -- Setting starting PID gains for left motor kp:%d ki:%d kd:%d", starting_l_kp, starting_l_ki, starting_l_kd);
	if(!robot.setPIDGains(starting_l_kp, starting_l_ki, starting_l_kd, miniQ::LeftStartingPID)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();

	ROS_INFO("miniQ -- Setting running PID gains for left motor kp:%d ki:%d kd:%d", running_l_kp, running_l_ki, running_l_kd);
	if(!robot.setPIDGains(running_l_kp, running_l_ki, running_l_kd, miniQ::LeftRunningPID)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();
	
	ROS_INFO("miniQ -- Setting starting PID gains for right motor kp:%d ki:%d kd:%d", starting_r_kp, starting_r_ki, starting_r_kd);
	if(!robot.setPIDGains(starting_r_kp, starting_r_ki, starting_r_kd, miniQ::RightStartingPID)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();

	ROS_INFO("miniQ -- Setting running PID gains for right motor kp:%d ki:%d kd:%d", running_r_kp, running_r_ki, running_r_kd);
	if(!robot.setPIDGains(running_r_kp, running_r_ki, running_r_kd, miniQ::RightRunningPID)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();

	ROS_INFO("miniQ -- Setting odometry callibration coefficients d:%lf yaw:%lf", odometry_d, odometry_yaw);
	if(!robot.setOdometryCallibration(odometry_d, odometry_yaw)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");

	ros::Duration(1.0).sleep();

	ROS_INFO("miniQ -- Setting gas sensor callibration coefficients a:%lf b:%lf", gas_a, gas_b);
	if(!robot.setGasSensorCallibration(gas_a, gas_b)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");		

	ros::Duration(1.0).sleep();
    
    ROS_INFO("miniQ -- Setting the timeout to %lf", timeout);
	if(!robot.setTimeout(timeout)) ROS_ERROR("miniQ -- ERROR!");
	else ROS_INFO("miniQ -- DONE.");	

	ROS_INFO("miniQ -- Setup was successfully! Goodbye!");

	return 0;
}

// EOF
