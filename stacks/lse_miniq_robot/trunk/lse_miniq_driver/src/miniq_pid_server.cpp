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
* Author: Gonçalo Cabrita on 20/08/2012
*********************************************************************/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <lse_miniq_driver/miniQConfig.h>

#include <lse_miniq_msgs/WheelVelocity.h>
#include "miniQ.h"

miniQ robot = miniQ();

// Parameters
bool l_s_updated;
int l_s_kp, l_s_ki, l_s_kd;
bool l_r_updated;
int l_r_kp, l_r_ki, l_r_kd;
bool r_s_updated;
int r_s_kp, r_s_ki, r_s_kd;
bool r_r_updated;
int r_r_kp, r_r_ki, r_r_kd;

bool turn_robot_off;
bool run_robot;

bool wheel_velocity_reference_updated;
double wheel_velocity_reference;

void callback(lse_miniq_driver::miniQConfig &config, uint32_t level)
{
	if(run_robot != config.run_robot && !config.run_robot) turn_robot_off = true;
	else turn_robot_off = false;
	run_robot = config.run_robot;
	
	if(wheel_velocity_reference == config.wheel_velocities) wheel_velocity_reference_updated = false;
	else
	{
		wheel_velocity_reference_updated = true;
		wheel_velocity_reference = config.wheel_velocities;
	}
	if(l_s_kp == config.left_starting_kp && l_s_ki == config.left_starting_ki && l_s_kd == config.left_starting_kd) l_s_updated = false;
	else
	{
		l_s_updated = true;
		l_s_kp = config.left_starting_kp;
		l_s_ki = config.left_starting_ki;
		l_s_kd = config.left_starting_kd;
	}  
	if(l_r_kp == config.left_running_kp && l_r_ki == config.left_running_ki && l_r_kd == config.left_running_kd) l_r_updated = false;
	else
	{
		l_r_updated = true;
		l_r_kp = config.left_running_kp;
		l_r_ki = config.left_running_ki;
		l_r_kd = config.left_running_kd;
	}
	if(r_s_kp == config.right_starting_kp && r_s_ki == config.right_starting_ki && r_s_kd == config.right_starting_kd) r_s_updated = false;
	else
	{
		r_s_updated = true;
		r_s_kp = config.right_starting_kp;
		r_s_ki = config.right_starting_ki;
		r_s_kd = config.right_starting_kd;
	}  
	if(r_r_kp == config.right_running_kp && r_r_ki == config.right_running_ki && r_r_kd == config.right_running_kd) r_r_updated = false;
	else
	{
		r_r_updated = true;
		r_r_kp = config.right_running_kp;
		r_r_ki = config.right_running_ki;
		r_r_kd = config.right_running_kd;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "miniq_pid_server");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");
    
  std::string port;
  pn.param<std::string>("port", port, "/dev/ttyUSB0");
  int baudrate;
  pn.param("baudrate", baudrate, 57600);
	
  ros::Publisher left_pub = n.advertise<lse_miniq_msgs::WheelVelocity>("/left_wheel_velocity", 20);
  ros::Publisher right_pub = n.advertise<lse_miniq_msgs::WheelVelocity>("/right_wheel_velocity", 20);
    
  if(!miniQ::openPort((char*)port.c_str(), baudrate))
  {
	ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
	ROS_BREAK();
  }
  ROS_INFO("miniQ PID Server -- Successfully connected to the miniQ!"); 
  
  ros::Duration(3.0).sleep();
    
  if(!robot.checkVersion())
  {
	ROS_FATAL("miniQ PID Server -- The firmware version of the miniQ robot is not compatible with this ROS node!");
	ROS_BREAK();
  }

  dynamic_reconfigure::Server<lse_miniq_driver::miniQConfig> server;
  dynamic_reconfigure::Server<lse_miniq_driver::miniQConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  turn_robot_off = false;

  ros::Rate r(2.5);
  while(n.ok())
  {
	if(!robot.updateWheelVelocities()) ROS_ERROR("miniQ PID Server -- Failed to update the miniQ wheel velocities!!!");
	else
	{
		lse_miniq_msgs::WheelVelocity right_msg;
		right_msg.reference = wheel_velocity_reference;
		right_msg.measured = robot.getRightWheelVelocity();
		right_pub.publish(right_msg);

		lse_miniq_msgs::WheelVelocity left_msg;
		left_msg.reference = wheel_velocity_reference;
		left_msg.measured = robot.getLeftWheelVelocity();
		left_pub.publish(left_msg);
	}

	if(l_s_updated)
	{
		l_s_updated = false;
		if(!robot.setPIDGains(l_s_kp, l_s_ki, l_s_kd, miniQ::LeftStartingPID)) ROS_ERROR("miniQ PID Server -- ERROR updating left starting PID!");
		ros::Duration(0.1).sleep();
	}

	if(l_r_updated)
	{
		l_r_updated = false;
		if(!robot.setPIDGains(l_r_kp, l_r_ki, l_r_kd, miniQ::LeftRunningPID)) ROS_ERROR("miniQ PID Server -- ERROR updating left running PID!");
		ros::Duration(0.1).sleep();
	}

	if(r_s_updated)
	{
		r_s_updated = false;
		if(!robot.setPIDGains(r_s_kp, r_s_ki, r_s_kd, miniQ::RightStartingPID)) ROS_ERROR("miniQ PID Server -- ERROR updating right starting PID!");
		ros::Duration(0.1).sleep();
	}

	if(r_r_updated)
	{	
		r_r_updated = false;
		if(!robot.setPIDGains(r_r_kp, r_r_ki, r_r_kd, miniQ::RightRunningPID)) ROS_ERROR("miniQ PID Server -- ERROR updating right running PID!");
		ros::Duration(0.1).sleep();
	}

	if(turn_robot_off)
	{
		turn_robot_off = false;
		robot.setVelocities(0.0, 0.0);
	}
	else if(run_robot && wheel_velocity_reference_updated)
	{
		wheel_velocity_reference_updated = false;
		robot.setVelocities(wheel_velocity_reference, 0.0);
        robot.updateVelocities();
	}
	
	ros::spinOnce();
	r.sleep();
  }
  return 0;
}
