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
* Author: Gonçalo Cabrita and Pedro Sousa on 17/11/2010
*********************************************************************/
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <lse_sensor_msgs/Nostril.h>
#include <roomba_500_series/Battery.h>
#include <geometry_msgs/Twist.h>
#include <fctuc_open_day/Poses.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

roomba_500_series::Battery battery;
std::vector<geometry_msgs::Pose2D> targets;
std::vector<geometry_msgs::Pose2D>::iterator target;

bool sendNewGoal;
bool needToBackUp;

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("Goal is complete.");
	// If the goal succeeded send a new one!
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
	// If it was aborted time to back up!
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;
}

void goalActiveCallback()
{
	ROS_INFO("Goal active! Hurray!");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	//ROS_INFO("Getting feedback! How cool is that?");
}

void noseCallback(const lse_sensor_msgs::Nostril::ConstPtr& nose)
{
	
}

void batteryCallback(const roomba_500_series::Battery::ConstPtr& batt)
{
	battery.level = batt->level;
	battery.dock = batt->dock;
}

void posesCallback(const fctuc_open_day::Poses::ConstPtr& poses)
{
	ROS_INFO("Got a new batch of poses!");
	targets = poses->poses;
	target = targets.begin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fctuc_open_day_demo");

	ROS_INFO("Welcome to FCTUC Open Day Demo!");
	
	ros::NodeHandle n;
	
	std::string tf_prefix;
	n.param<std::string>("tf_prefix", tf_prefix, "");
	
	ros::Publisher cmd_vel_pub  = n.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
	
	ros::Subscriber nose_sub = n.subscribe("/nose", 20, noseCallback);
	ros::Subscriber batt_sub = n.subscribe("/battery", 20, batteryCallback);
	ros::Subscriber pose_sub = n.subscribe("/poses", 2, posesCallback);
	
	ros::Rate r(10);
	ROS_INFO("Waiting for the first batch of poses to be published...");
	while(targets.size()==0 && ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("Got it!");
	
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the move_base action server to come online...");
	if(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_FATAL("Forgot to launch move_base now did we?");
		ROS_BREAK();
	}
	ROS_INFO("Found it!");
	
	int backUpCounter = 0;
	sendNewGoal = true;
	needToBackUp = false;
	
	target = targets.begin();
	while(ros::ok())
	{	
		// If we are trapped or we are recharged back up a lil bit
		if(needToBackUp || (battery.level>80.0 && battery.dock))
		{
			needToBackUp = true;
			sendNewGoal = false;
			if(backUpCounter==0)
			{
				ROS_INFO("The wall is too close! I need to do some backing up...");
				// Move the robot back...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = -0.05;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel);
			}
			
			if(backUpCounter==40)
			{
				// Turn the robot around...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.5;
				cmd_vel_pub.publish(cmd_vel);
			}
			
			if(backUpCounter==100)
			{
				// Stop the robot...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel);
				
				ROS_INFO("Done backing up, now on with my life!");
				
				// Reset variables
				backUpCounter = 0;
				sendNewGoal = true;
				needToBackUp = false;
			}
			backUpCounter++;
		}
	
		if(sendNewGoal)
		{
			std::string frame_name = tf_prefix;
			move_base_msgs::MoveBaseGoal goal;
		
			// Send a goal to the robot
			goal.target_pose.header.frame_id = frame_name.append("/map");
			goal.target_pose.header.stamp = ros::Time::now();
		
			// Send the next goal
			goal.target_pose.pose.position.x = target->x;
			goal.target_pose.pose.position.y = target->y;
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target->theta);
			
			target++;
			if(target==targets.end()) target=targets.begin();
			
			sendNewGoal = false;
			ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
		}
		
		// If you smell something do interactive stuff!
		//if()
		
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting FCTUC Open Day Demo...");

	return 0;
}
