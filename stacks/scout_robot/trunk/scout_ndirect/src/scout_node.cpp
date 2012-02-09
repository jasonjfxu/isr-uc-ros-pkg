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
* Author: Gonçalo Cabrita and Pedro Sousa on 21/10/2010
*********************************************************************/
#define NODE_VERSION 0.02

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <lse_sensor_msgs/Range.h>		// sonar

#include "Nclient.h"

#define INCH_TO_METER		0.0254
#define DEGREE_TO_RADIAN	0.0174532925

#define SCOUT_BAUD_RATE		38400
#define BRIDGE_BAUD_RATE	19200

#define BATTERY_LOW 	0
#define BATTERY_MED 	1
#define BATTERY_HIGH 	2

long State[NUM_STATE];

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	scout_vm((int)(cmd_vel->linear.x/(INCH_TO_METER)*10), (int)(cmd_vel->angular.z/(DEGREE_TO_RADIAN)*10));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "scout_node");

	ROS_INFO("Scout for ROS %.2f", NODE_VERSION);

	ros::NodeHandle n("~");
	
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Publisher sonar_pub = n.advertise<lse_sensor_msgs::Range>("/sonar", 50);
	
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);

	std::string port;
	n.param<std::string>("port", port, "/dev/ttyUSB0");
	
	std::string model;
	n.param<std::string>("model", model, "Scout2");
	
	int robot_model;
	if(model.compare("N200")==0) robot_model = MODEL_N200;
	else if(model.compare("N150")==0) robot_model = MODEL_N150;
	else if(model.compare("Scout")==0) robot_model = MODEL_SCOUT;
	else if(model.compare("Scout2")==0) robot_model = MODEL_SCOUT2;
	else
	{
		ROS_FATAL("Scout -- Unknown robot model: %s! Options are N200, N150, Scout and Scout2", model.c_str());
		ROS_BREAK();
	}

	// Connect to the Scout
	connect_robot(1, robot_model, port.c_str(), SCOUT_BAUD_RATE);
	// Reset
	dp(0,0);
	da(0,0);
	zr();
	// Set command timeout
	//conf_tm(1000);

	// Configure sonars
  	int sn_order[16] = {0, 2, 15, 1, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7, 9, 8};
  	conf_sn(15, sn_order);
	
	ros::Time current_time;
	
	ros::Rate r(10.0);
	while(n.ok())
	{
		// Update the Scout readings
		gs();
		
		current_time = ros::Time::now();

		//ROS_INFO("Error: %d", State[STATE_ERROR]);
		
		float odom_x = State[STATE_CONF_X]*10*INCH_TO_METER;
		float odom_y = State[STATE_CONF_Y]*10*INCH_TO_METER;
		float odom_yaw = State[STATE_CONF_STEER]*10*DEGREE_TO_RADIAN;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";
		
		odom_trans.transform.translation.x = odom_x;
		odom_trans.transform.translation.y = odom_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "/odom";
		
		//set the position
		odom.pose.pose.position.x = odom_x;
		odom.pose.pose.position.y = odom_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "/base_link";
		odom.twist.twist.linear.x = State[STATE_VEL_TRANS]*INCH_TO_METER/10;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = State[STATE_VEL_STEER]*DEGREE_TO_RADIAN/10;
		
		//publish the message
		odom_pub.publish(odom);
		
		//now on to the sonar array!
		for(int STATE_SONAR_i=STATE_SONAR_0 ; STATE_SONAR_i<=STATE_SONAR_15 ; STATE_SONAR_i++)
		{
			lse_sensor_msgs::Range sonar;
			sonar.header.stamp = current_time;
			char frame_id[32];
			sprintf(frame_id, "/base_sonar_%d", STATE_SONAR_i-STATE_SONAR_0);
			sonar.header.frame_id = frame_id;
			sonar.field_of_view = 0.34906585;	// 20 degress
			sonar.min_range = 0.50;
			sonar.max_range = 5.00;
			sonar.range = (float)(State[STATE_SONAR_i]*INCH_TO_METER);
			sonar_pub.publish(sonar);
		}
		
		char status = State[STATE_MOTOR_STATUS];
		
		bool left_wheel_moving = status & 0x01;
		bool right_wheel_moving = (status>>1) & 0x01;
		
		int battery_state;
		bool b0 = (status>>2) & 0x01;
		bool b1 = (status>>3) & 0x01;
		if(!b0 && !b1) battery_state = BATTERY_LOW;
		else if(b0 && !b1) battery_state = BATTERY_MED;
		else if(!b0 && b1) battery_state = BATTERY_HIGH;
		
		bool is_plugged = (status>>4) & 0x01;
		bool is_charging = (status>>5) & 0x01;
		bool emergency_stop = (status>>6) & 0x01;

		ros::spinOnce();
		r.sleep();
	}
	
	// Turn sonars off
  	int sn_off[1] = {255};
  	conf_sn(15, sn_off);
	// Stop the robot
	st();
	// Disconnect the Scout
	disconnect_robot(1);
}

// EOF
