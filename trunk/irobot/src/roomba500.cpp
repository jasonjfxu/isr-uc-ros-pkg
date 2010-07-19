/*
 *  roomba500.cpp
 *  
 *
 *  Created by Gonçalo Cabrita on 21/05/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	NA
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <irobot/Battery.h>

#include "iRobot_OI/iRobot_OI.h"


iRobot_OI roomba("/dev/ttyUSB0");


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	roomba.drive(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "roomba500_node");
	
	double last_x, last_y, last_yaw;
	double vel_x, vel_y, vel_yaw;
	double dt;
	float lastCharge;
	int timeRemaining = -1;
	
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher bat_pub = n.advertise<irobot::Battery>("battery", 50);
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Subscriber cmd_vel_sub;
	cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelReceived);
	
	OI_Packet_ID sensorPackets[5] = {OI_PACKET_RIGHT_ENCODER, OI_PACKET_LEFT_ENCODER, OI_PACKET_BATTERY_CHARGE, OI_PACKET_BATTERY_CAPACITY, OI_PACKET_CHARGE_SOURCES};

	roomba.setSensorPackets(sensorPackets, 5, OI_PACKET_RIGHT_ENCODER_SIZE + OI_PACKET_LEFT_ENCODER_SIZE + OI_PACKET_BATTERY_CHARGE_SIZE + OI_PACKET_BATTERY_CAPACITY_SIZE + OI_PACKET_CHARGE_SOURCES_SIZE);

	if( roomba.openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
	else ROS_ERROR("Could not connect to Roomba.");
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(1.0);
	while(n.ok())
	{
		current_time = ros::Time::now();
		
		last_x = roomba.odometryX;
		last_y = roomba.odometryY;
		last_yaw = roomba.odometryYaw;
		
		if( roomba.getSensorPackets(1000) == -1) ROS_ERROR("Could not retrieve sensor packets.");
		else roomba.calculateOdometry();
		
		dt = (current_time - last_time).toSec();
		vel_x = (roomba.odometryX - last_x)/dt;
		vel_y = (roomba.odometryY - last_y)/dt;
		vel_yaw = (roomba.odometryYaw - last_yaw)/dt;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(roomba.odometryYaw);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = roomba.odometryX;
		odom_trans.transform.translation.y = roomba.odometryY;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		//set the position
		odom.pose.pose.position.x = roomba.odometryX;
		odom.pose.pose.position.y = roomba.odometryY;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
		
		//publish the message
		odom_pub.publish(odom);

		//next, we'll publish the battery message over ROS
		irobot::Battery battery;
		battery.header.stamp = current_time;

		battery.charging_source = "not_charging";
		battery.level = 100.0*(roomba.charge/roomba.capacity);
		if(lastCharge > roomba.charge) timeRemaining = (int)(battery.level/((lastCharge-roomba.charge)/roomba.capacity)/dt)/60;
		battery.time_remaining = timeRemaining;
		
		//publish the message
		bat_pub.publish(battery);

		last_time = current_time;
		lastCharge = roomba.charge;

		ros::spinOnce();
		r.sleep();
	}
	
	roomba.powerDown();
	roomba.closeSerialPort();
}

// EOF
