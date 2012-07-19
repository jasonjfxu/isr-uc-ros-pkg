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
* Author: Gonçalo Cabrita on 28/05/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include "RobChair.h"

RobChair robot;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    robot.setVelocities(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robchair_node");

	ROS_INFO("RobChair for ROS");

    ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelReceived);
    
    ros::Publisher tx_pub = n.advertise<can_msgs::CANFrame>("/can_bus_tx", 10);
    ros::Subscriber rx_sub = n.subscribe<can_msgs::CANFrame>("/can_bus_rx", 10, &RobChair::receivedCANFrame, &robot);
    
    // PI parameters...
    // If we defined Kp and Ki set them
    double kp, ki;
	if(pn.getParam("kp", kp) && pn.getParam("ki", ki))
    {
        ROS_INFO("RobChair -- Setting new PI gains.");
        robot.initializeRobChair(&tx_pub, kp, ki);
    }
    // Otherwise go with default values
    else
    {
        robot.initializeRobChair(&tx_pub);
    }
    
	ros::Time current_time;
	
	ros::Rate r(50.0);
	while(n.ok())
	{
        current_time = ros::Time::now();
        
        float odom_x = robot.getPositionX();
        float odom_y = robot.getPositionY();
        float odom_yaw = robot.getYaw();
		
		// Since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
		
		// First, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";
		
		odom_trans.transform.translation.x = odom_x;
		odom_trans.transform.translation.y = odom_y;
		odom_trans.transform.rotation = odom_quat;
		
		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		// Next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "/odom";
		
		// Set the position
		odom.pose.pose.position.x = odom_x;
		odom.pose.pose.position.y = odom_y;
		odom.pose.pose.orientation = odom_quat;
		
		// Set the velocity
		odom.child_frame_id = "/base_link";
		odom.twist.twist.linear.x = robot.getLinearVelocity();
		odom.twist.twist.angular.z = robot.getAngularVelocity();
		
		// Publish the message
		odom_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
