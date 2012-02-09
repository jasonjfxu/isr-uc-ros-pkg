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
* Author: Gonçalo Cabrita on 5/1/2011
*********************************************************************/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "PPExplorer.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool send_goal;
bool goal_failed;

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("PPExplorer Node - goalDoneCallback - Goal is complete.");
	// If the goal succeeded send a new one!
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) send_goal = true;
	// If it was aborted time to back up!
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) goal_failed = true;
}

void goalActiveCallback()
{
	//ROS_INFO("PPExplorer - goalActiveCallback - Node Goal active! Hurray!");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	//ROS_INFO("PPExplorer Node - goalFeedbackCallback - Getting feedback! How cool is that?");
}

void sendRobotToGoal(MoveBaseClient * ac, geometry_msgs::PoseStamped * robot_goal)
{
	move_base_msgs::MoveBaseGoal goal;
			
	goal.target_pose.header.frame_id = robot_goal->header.frame_id;
	goal.target_pose.header.stamp = robot_goal->header.stamp;

	goal.target_pose.pose.position.x = robot_goal->pose.position.x;
	goal.target_pose.pose.position.y = robot_goal->pose.position.y;
	goal.target_pose.pose.orientation = robot_goal->pose.orientation;
	
	ROS_INFO("Sending robot to %lf %lf", robot_goal->pose.position.x, robot_goal->pose.position.y);

	ac->sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "pp_explorer_node");
  	
  	ROS_INFO("ParticlePlumeExplorer for ROS v1.0");

	particle_plume::PPExplorer pp;
	
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	// Wait for the action server to come up
	ROS_INFO("PPExplorer Node -- Waiting for the move_base action server to come online...");
	bool ac_online = false;
	for(int i=0 ; i<3 ; i++)
	{
		if(ac.waitForServer(ros::Duration(1.0)))
		{
			ac_online = true;
			break;
		}
	
		ROS_INFO("PPExplorer Node -- Unable to find the move_base action server, retrying...");
	}
	if(!ac_online)
	{
		ROS_FATAL("PPExplorer Node -- Forgot to launch move_base now did we?");
		ROS_BREAK();
		return(-1);
	}
	ROS_INFO("PPExplorer Node -- Found it!");
	
	ROS_INFO("PPExplorer Node -- Waiting for a map msg to arrive...");
	while(!pp.isReady())
	{
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("PPExplorer Node -- Got it!");
	
	geometry_msgs::PoseStamped robot_goal;
	
	send_goal = true;
	goal_failed = false;
	
	ros::Time start_time = ros::Time::now();
	
	ros::Rate r(1);
	while(ros::ok())
	{
		if(send_goal)
		{
			send_goal = false;
			if(pp.findYumiestSlice(&robot_goal))
			{
				sendRobotToGoal(&ac, &robot_goal);
			}
			else if(pp.findClearing(&robot_goal))
			{
				ROS_WARN("PPExplorer Node -- All slices were discarded! Re-locating the robot...");
				sendRobotToGoal(&ac, &robot_goal);
			}
			else
			{
				ROS_INFO("PPExplorer Node -- Finished!");
				ros::Duration time_spent = ros::Time::now() - start_time;
				ROS_INFO("PPExplorer Node -- Took %lf seconds to complete the exploration! Got %lf%% of explored area!", time_spent.toSec(), pp.getExploredArea()*100.0);
				return(0);
			}
		}
		
		if(goal_failed)
		{
			goal_failed = false;
			if(pp.nextYumiestSlice(&robot_goal))
			{
				sendRobotToGoal(&ac, &robot_goal);
			}
			else if(pp.findClearing(&robot_goal))
			{
				ROS_WARN("PPExplorer Node -- All slices turned out being bad goals! Re-locating the robot...");
				sendRobotToGoal(&ac, &robot_goal);
			}
			else
			{
				ROS_INFO("PPExplorer Node -- Finished!");
				ros::Duration time_spent = ros::Time::now() - start_time;
				ROS_INFO("PPExplorer Node -- Took %lf seconds to complete the exploration! Got %lf%% of explored area!", time_spent.toSec(), pp.getExploredArea()*100.0);
				return(0);
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
	
 	return(0);
}

// EOF

