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
* Author: Gonçalo Cabrita on 03/09/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <lse_sensor_msgs/Nostril.h>		// nose

#include "miniQ.h"

// Robot data structure
class Robot
{
    public:
    Robot(int id) : robot(), odom_pub(), cmd_vel_sub(), prefix(), nose_pub()
    {
        robot.setId(id);

        prefix = "/robot_";
        prefix.append<int>(1, 48+id);
    };
    
    Robot(const Robot& r) : robot(), odom_pub(), cmd_vel_sub(), prefix(), nose_pub()
    {
        robot = r.robot;

	odom_pub = r.odom_pub;
	cmd_vel_sub = r.cmd_vel_sub;
	prefix = r.prefix;
	nose_pub = r.nose_pub;
    };
    
    // Robot
    miniQ robot;
    
    ros::Publisher odom_pub;
    ros::Subscriber cmd_vel_sub;

    ros::Publisher nose_pub;

    std::string prefix;
};

// The group of miniQs!!!
std::vector<Robot> miniqs;


void cmdVelReceived(int index, const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    miniqs[index].robot.setVelocities(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "miniq_node");

	ROS_INFO("miniQ for ROS - Multi robot version.");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
	pn.param("baudrate", baudrate, 57600);

    	if(!miniQ::openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Successfully connected to the miniQ!");

    	std::vector<int> ids;
	
        // Lets load the list of robot ids...
    	XmlRpc::XmlRpcValue list_of_ids;
    	if( n.getParam("/list_of_ids", list_of_ids) )
    	{   
        	ROS_ASSERT(list_of_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<list_of_ids.size() ; ++i) 
		{
		    ROS_ASSERT(list_of_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    ids.push_back(static_cast<int>(list_of_ids[i]));
		}
    	}
    	else
    	// If a list of ids is not defined scan for robots...
    	{
		ROS_INFO("miniQ -- A list of IDs was not provided, scanning for robots...");
        
		for(int i=0 ; i<=10 ; i++)
		{
		    int id = miniQ::scanForId(i);
		    if(id>0)
		    {
				ids.push_back(id);
				ROS_INFO("miniQ -- Found id %d", id);
		    }  
		}
    	}
	if(ids.size() == 0)
	{
		ROS_FATAL("miniQ -- Could not find any miniQs!!!");
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Finished scanning for robots!");
	    
	for(int i=0 ; i<ids.size() ; ++i) 
	{
		miniqs.push_back(Robot(ids[i]));

		std::string odom_topic = miniqs[i].prefix;
		odom_topic.append("/odom");
		miniqs[i].odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 50);

		std::string cmd_vel_topic = miniqs[i].prefix;
		cmd_vel_topic.append("/cmd_vel");
		miniqs[i].cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(cmdVelReceived, i, _1) );

		std::string nose_topic = miniqs[i].prefix;
		nose_topic.append("/nose");
		miniqs[i].nose_pub = n.advertise<lse_sensor_msgs::Nostril>(nose_topic, 20);
	}

	tf::TransformBroadcaster odom_broadcaster;
    
	ros::Time current_time;

	std::string frame_id;
	
	ros::Rate r(MINIQ_RATE);
	while(n.ok())
	{
        	current_time = ros::Time::now();
        
		for(int i=0 ; i<miniqs.size() ; i++)
       	 	{
			ros::Time start_time = ros::Time::now();
			if(!miniqs[i].robot.update()) ROS_WARN("miniQ -- Failed to update the data for robot %d!!!", miniqs[i].robot.getID());

			ros::Duration elapsed_time = ros::Time::now() - start_time;
			//ROS_INFO("Getting the odometry for robot %d took %lf sec.", miniqs[i].robot.getID(), elapsed_time.toSec());
			
			double odom_x = miniqs[i].robot.getX();
			double odom_y = miniqs[i].robot.getY();
			double odom_yaw = miniqs[i].robot.getYaw();

			//ROS_INFO("Publishing data... %lf %lf %lf", odom_x, odom_y, odom_yaw);
	
			// Since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
	
			// First, we'll publish the transform over tf
			tf::Transform new_tf(tf::createQuaternionFromYaw(odom_yaw), tf::Vector3(odom_x, odom_y, 0.0));
			ros::Time transform_expiration = current_time + ros::Duration(1/(MINIQ_RATE)*2.0);
			std::string odom_frame_id = miniqs[i].prefix;
			odom_frame_id.append("/odom");
			std::string base_frame_id = miniqs[i].prefix;
			base_frame_id.append("/base_link");
			tf::StampedTransform odom_trans(new_tf, transform_expiration, odom_frame_id, base_frame_id);
	
			// Send the transform
			odom_broadcaster.sendTransform(odom_trans);
	
			// Next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			frame_id = miniqs[i].prefix;
			frame_id.append("/odom");
			odom.header.frame_id = frame_id;
	
			// Set the position
			odom.pose.pose.position.x = odom_x;
			odom.pose.pose.position.y = odom_y;
			odom.pose.pose.orientation = odom_quat;
	
			// Set the velocity
			frame_id = miniqs[i].prefix;
			frame_id.append("/base_link");
			odom.child_frame_id = frame_id;
			odom.twist.twist.linear.x = miniqs[i].robot.getLinearVelocity();
			odom.twist.twist.angular.z = miniqs[i].robot.getAngularVelocity();
	
			// Publish the message
			miniqs[i].odom_pub.publish(odom);

			// Nose data...
			lse_sensor_msgs::Nostril nose_msg; 
	
			nose_msg.header.stamp = current_time;
			nose_msg.header.frame_id = frame_id;

			nose_msg.sensor_model = "e2v MiCS 5524";
			nose_msg.reading = miniqs[i].robot.getGas();
			nose_msg.min_reading = 0.0;
			nose_msg.max_reading = 1.0;
			nose_msg.clean_air = 0.0;
			nose_msg.raw_data = miniqs[i].robot.getRawGas();

			miniqs[i].nose_pub.publish(nose_msg);

			ros::Duration((1/MINIQ_RATE)/(double)(miniqs.size())-elapsed_time.toSec()*2.0).sleep();
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

// EOF
