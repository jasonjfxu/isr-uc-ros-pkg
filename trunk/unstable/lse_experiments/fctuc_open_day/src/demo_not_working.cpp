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
#include <roomba_500_series/GoDockAction.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool nextGoal;
bool needToBackUp;

roomba_500_series::Battery battery;

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("Goal is done!");
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) nextGoal = true;
	else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;
}

void goalActiveCallback()
{
	//ROS_INFO("Goal active! Hurray!");
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fctuc_open_day_demo");

	ROS_INFO("Welcome to FCTUC Open Day Demo!");
	
	ros::NodeHandle n;
	
	//ros::Publisher cmd_vel_pub  = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	
	ros::Subscriber nose_sub = n.subscribe("/nose", 20, noseCallback);
	ros::Subscriber batt_sub = n.subscribe("/battery", 20, batteryCallback);
	
	/*actionlib::SimpleActionClient<roomba_500_series::GoDockAction> gd("godock", true);
	ROS_INFO("Waiting for the godock action server to come online... Connecting the roomba would be a good starting point!");
	gd.waitForServer();
	ROS_INFO("Found the godock action server! Moving on...");*/
	
	std::string tf_prefix;
	n.param<std::string>("tf_prefix", tf_prefix, "");
	
	std::string file_path;
	n.param<std::string>("demo/file_name", file_path, "demo.goal");
	
	FILE * goalsFile;
	goalsFile = fopen(file_path.c_str(), "r");
	if(goalsFile == NULL)
	{
		ROS_INFO("Cannot open file %s", file_path.c_str());
		ROS_BREAK();	
	}
	
	std::vector<geometry_msgs::Pose2D> targets;
	std::vector<geometry_msgs::Pose2D>::iterator target;
	
	/*geometry_msgs::Pose2D myDock;
	myDock.x = 1.0;
	myDock.y = 1.0;
	myDock.theta = 0.0;*/
	
	geometry_msgs::Pose2D pose;
	// Might come in handy... some day :P
	float z, qx, qy, qz, qw;
	while(!feof(goalsFile))
	{
		fscanf(goalsFile, "Frame:/map, Position(%f, %f, %f), Orientation(%f, %f, %f, %f) = Angle: %lf\n", &(pose.x), &(pose.y), &z, &qx, &qy, &qz, &qw, &(pose.theta));
		targets.push_back(pose);
	}
	
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the move_base action server to come online... Forgot to launch move_base now did we?");
	while(!ac.waitForServer(ros::Duration(5.0))){;}
	ROS_INFO("Found the move_base action server! Moving on...");

	battery.level = 100.0;
	battery.dock = false;
	needToBackUp = false;
	nextGoal = false;
	bool sendGoal = true;
	bool docking = false;
	int backUpCounter = 0;
	target = targets.begin();
	ROS_INFO("Lets have some fun...");
	
	ros::Rate r(10);
	while(ros::ok())
	{	
		// If we are trapped or we are recharged back up a lil bit
		/*if(needToBackUp)// || (battery.level>80.0 && battery.dock))
		{
			needToBackUp = true;
			sendGoal = false;
			nextGoal = false;
			if(backUpCounter==0)
			{
				ROS_INFO("The wall is too close! I need to do some backing up...");
				// Move the robot back...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = -0.1;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel);
			}
			
			if(backUpCounter==50)
			{
				// Turn the robot around...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 1.0;
				cmd_vel_pub.publish(cmd_vel);
			}
			
			if(backUpCounter==80)u
			{
				// Stop the robot...
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel);
				
				ROS_INFO("Done backing up, now on with my life!");
				
				// Reset variables
				backUpCounter = 0;
				sendGoal = true;
				needToBackUp = false;
			}
			
			backUpCounter++;
		}*/
	
		// Send the next goal!
		if(nextGoal)
		{
			// Run the docking proceadure!
			if(docking)
			{
				ROS_INFO("Lets see if I can bullseye the dock...");
				
				/*ac.cancelAllGoals();
			
				roomba_500_series::GoDockGoal goal;
				goal.timeout = ros::Duration(30.0);
				gd.sendGoal(goal);
			
				gd.waitForResult(ros::Duration(30.0));

				if(!battery.dock) ROS_ERROR("I could not dock! No I'm going to run out of battery and fade away! Help!!!");
				else ROS_INFO("Got it! And now for some tasty battery juice!");*/
				docking = false;
			}
			else
			{
				target++;
				if(target==targets.end()) target=targets.begin();
			}
			sendGoal = true;
			nextGoal = false;
		}
	
		// Send a goal to the action client
		if(sendGoal || (battery.level<50.0 && !battery.dock && !docking))
		{
			std::string frame_name = tf_prefix;
			move_base_msgs::MoveBaseGoal goal;
		
			// Send a goal to the robot
			goal.target_pose.header.frame_id = frame_name.append("/map");
			goal.target_pose.header.stamp = ros::Time::now();
		
			// If we need to recharge
			if(battery.level<50.0 && !battery.dock)
			{
				/*ROS_INFO("Going for docking station... Battery is at %.0f%%", battery.level);
				goal.target_pose.pose.position.x = myDock.x;
				goal.target_pose.pose.position.y = myDock.y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(myDock.theta);
				docking = true;*/
			}
			else
			{
				ROS_INFO("Sending next goal... How long do we have to keep this up?");
				goal.target_pose.pose.position.x = target->x;
				goal.target_pose.pose.position.y = target->y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target->theta);
			}
			
			sendGoal = false;
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
