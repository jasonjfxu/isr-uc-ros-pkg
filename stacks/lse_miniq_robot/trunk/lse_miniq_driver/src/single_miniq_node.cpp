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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <lse_sensor_msgs/Nostril.h>		// nose

#include "miniQ.h"

miniQ robot = miniQ();

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    robot.setVelocities(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "miniq_node");

	ROS_INFO("miniQ for ROS - Single robot version.");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
	pn.param("baudrate", baudrate, 57600);
	
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelReceived);

	ros::Publisher nose_pub = n.advertise<lse_sensor_msgs::Nostril>("/nose", 50);
    
	if(!miniQ::openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Successfully connected to the miniQ!");
    
    	ros::Duration(0.5).sleep();
    
    	if(!robot.checkVersion())
	{
		ROS_FATAL("miniQ -- The firmware version of the miniQ robot is not compatible with this ROS node!");
		ROS_BREAK();
	}
    
	ros::Time current_time;
	
	ros::Rate r(MINIQ_RATE);
	while(n.ok())
	{
        current_time = ros::Time::now();
        
        if(!robot.update()) ROS_WARN("miniQ -- Failed to update the data!!!");
	else
	{
        	double odom_x = robot.getX();
        	double odom_y = robot.getY();
        	double odom_yaw = robot.getYaw();

		//ROS_INFO("Publishing data... %lf %lf %lf", odom_x, odom_y, odom_yaw);
		
		// Since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
		
		// First, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = odom_x;
		odom_trans.transform.translation.y = odom_y;
		odom_trans.transform.rotation = odom_quat;
		
		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		// Next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		// Set the position
		odom.pose.pose.position.x = odom_x;
		odom.pose.pose.position.y = odom_y;
		odom.pose.pose.orientation = odom_quat;
		
		// Set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = robot.getLinearVelocity();
		odom.twist.twist.angular.z = robot.getAngularVelocity();
		
		// Publish the message
		odom_pub.publish(odom);

		// Nose data...
		lse_sensor_msgs::Nostril nose_msg; 
		
		nose_msg.header.stamp = current_time;
		nose_msg.header.frame_id = "base_link";

		nose_msg.sensor_model = "e2v MiCS 5524";
		nose_msg.reading = robot.getGas();
		nose_msg.min_reading = 0.0;
		nose_msg.max_reading = 1.0;
		nose_msg.clean_air = 0.0;
		nose_msg.raw_data = robot.getRawGas();

		nose_pub.publish(nose_msg);
	}

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
