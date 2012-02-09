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
* Author: Gonçalo Cabrita on 05/03/2011
*********************************************************************/

#include <ros/ros.h>
#include <angles/angles.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <vector>
#include <list>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RandomWalk
{
	public:
	RandomWalk();
	~RandomWalk();
	
	private:
	//! Data structure that holds the obstacle cells.
	nav_msgs::GridCells obstacles_;
	//! Data structure that holds the inflated obstacle cells.
	nav_msgs::GridCells inflated_;
	
	//! Map
	nav_msgs::OccupancyGrid map_;
	
	//! Robot pose on the global frame referential
	geometry_msgs::PoseStamped robot_pose_;
	
	//! Node handler.
	ros::NodeHandle n_;
	
	//! Map subscriber.
	ros::Subscriber map_sub_;
	//! Obstacles sunscriber.
	ros::Subscriber obstacles_sub_;
	//! Inflated obstacles subscriber.
	ros::Subscriber inflated_sub_;
	//! Visited cells topic subscriber.
	ros::Subscriber cells_sub_;
	
	//! Odometry topic subscriber.
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
	
	//! Data structure that holds the visited cells.
	nav_msgs::GridCells cells_;
	
	//! Number of known cells on the map.
	int map_known_cells_;
	
	//! Move Base Action Server.
	MoveBaseClient ac_;
	
	//! Inflation radius from the nav stack.
	double inflation_radius_;
	
	ros::Time start_time_;
	
	void setGoal();
	bool checkGoal(move_base_msgs::MoveBaseGoal * goal);
	bool checkCell(int goal_cell_x, int goal_cell_y);
	
	void odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg);
	void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg);
	void inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void cellsCallback(const nav_msgs::GridCells::ConstPtr& msg);
	
	void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
	void goalActiveCallback();
	void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
};

RandomWalk::RandomWalk() : ac_("move_base", true)
{
	ROS_INFO("RandomWalk -- Waiting for the move_base action server to come online...");
	bool ac_online = false;
	for(int i=0 ; i<3 ; i++)
	{
		if(ac_.waitForServer(ros::Duration(1.0)))
		{
			ac_online = true;
			break;
		}
	
		ROS_INFO("RandomWalk -- Unable to find the move_base action server, retrying...");
	}
	if(!ac_online)
	{
		ROS_FATAL("RandomWalk -- Forgot to launch move_base now did we?");
		ROS_BREAK();
		return;
	}
	ROS_INFO("RandomWalk -- Found it!");
	
	// Subscribe to topics
	odom_sub_.subscribe(n_, "odom", 10);
	tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, "map", 10);
    tf_filter_->registerCallback( boost::bind(&RandomWalk::odomCallback, this, _1) );
    
    map_sub_ = n_.subscribe("map", 1, &RandomWalk::mapCallback, this);
	obstacles_sub_ = n_.subscribe("move_base/local_costmap/obstacles", 1, &RandomWalk::obstaclesCallback, this);
	inflated_sub_ = n_.subscribe("move_base/local_costmap/inflated_obstacles", 1, &RandomWalk::inflatedCallback, this);
	cells_sub_ = n_.subscribe("/visited_cells", 1, &RandomWalk::cellsCallback, this);
	
	n_.param("move_base/local_costmap/inflation_radius", inflation_radius_, 0.20);
	
	map_known_cells_ = 0;
	
	start_time_ = ros::Time::now();
}

RandomWalk::~RandomWalk()
{
	ros::Duration time_spent = ros::Time::now() - start_time_;
	ROS_INFO("RandomWalk -- Took %lf seconds to complete the exploration! Got %lf%% of explored area!", time_spent.toSec(), (cells_.cell_height*cells_.cell_width*cells_.cells.size())/(map_.info.resolution*map_.info.resolution*map_known_cells_));
}

void RandomWalk::setGoal()
{
	move_base_msgs::MoveBaseGoal goal;
	
	int x, y;

	do
	{
		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();
	
		/*double yaw = (rand() % 628 - 314)/100.0;
		double step = (rand() % 75 + 25)/100.0;

		goal.target_pose.pose.position.x = robot_pose_.pose.position.x + step * cos(tf::getYaw(robot_pose_.pose.orientation) + yaw);
		goal.target_pose.pose.position.y = robot_pose_.pose.position.y + step * sin(tf::getYaw(robot_pose_.pose.orientation) + yaw);
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(tf::getYaw(robot_pose_.pose.orientation) + yaw));*/
		
		x = rand() % map_.info.width;
		y = rand() % map_.info.height;
		
		goal.target_pose.pose.position.x = x*map_.info.resolution;
		goal.target_pose.pose.position.y = y*map_.info.resolution;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		
		//ROS_INFO("RandomWalk - %s - Testing %lf %lf...", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		
		if(!n_.ok())
		{
			ROS_INFO("RandomWalk - %s - Exiting...", __FUNCTION__);
			return;
		}
	}
	while(!checkCell(x, y));
	//while(!checkGoal(&goal));

	ROS_INFO("RandomWalk - %s - Sending robot to %lf %lf", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

	ac_.sendGoal(goal, boost::bind(&RandomWalk::goalDoneCallback, this,  _1, _2), boost::bind(&RandomWalk::goalActiveCallback, this), boost::bind(&RandomWalk::goalFeedbackCallback, this, _1));
}

bool RandomWalk::checkGoal(move_base_msgs::MoveBaseGoal * goal)
{
	int goal_cell_x = goal->target_pose.pose.position.x / map_.info.resolution;
	int goal_cell_y = goal->target_pose.pose.position.y / map_.info.resolution;

	// 1. If goal is out of bounds, discard it!
	if(goal_cell_x > map_.info.width-1 || goal_cell_x < 0 || goal_cell_y > map_.info.height-1 || goal_cell_y < 0)
	{
		return false;
	}
	
	if(!checkCell(goal_cell_x, goal_cell_y)) return false;
	
	return true;
}

bool RandomWalk::checkCell(int goal_cell_x, int goal_cell_y)
{
	std::vector<geometry_msgs::Point>::iterator obstacle;

	// 1. If goal is an obstacle or too close to one on the map, discard it!
	if(map_.data[goal_cell_y*map_.info.width+goal_cell_x] != 0) return false;
					
	int cir = ceil(inflation_radius_/map_.info.resolution)+1;
	for(int i=(goal_cell_x-cir<0 ? 0 : goal_cell_x-cir) ; i<(goal_cell_x+cir>map_.info.width ? map_.info.width : goal_cell_x+cir) ; i++)
	{
		for(int j=(goal_cell_y-cir<0 ? 0 : goal_cell_y-cir) ; j<(goal_cell_y+cir>map_.info.height ? map_.info.height : goal_cell_y+cir) ; j++)
		{
			if(map_.data[j*map_.info.width+i] != 0 && sqrt((goal_cell_x-i)*(goal_cell_x-i)+(goal_cell_y-j)*(goal_cell_y-j)) <= (float)cir-1.0)
			{
				return false;
			}
		}
	}
	
	// 2. If goal is too close to an obstacle, discard it!
	for(obstacle = obstacles_.cells.begin() ; obstacle != obstacles_.cells.end() ; obstacle++)
	{
		if(goal_cell_x >= obstacle->x && goal_cell_x <= obstacle->x+obstacles_.cell_width && goal_cell_y >= obstacle->y && goal_cell_y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	
	// 3. If goal is too close to an inflated obstacle, discard it!
	for(obstacle = inflated_.cells.begin() ; obstacle != inflated_.cells.end() ; obstacle++)
	{
		if(goal_cell_x >= obstacle->x && goal_cell_x <= obstacle->x+obstacles_.cell_width && goal_cell_y >= obstacle->y && goal_cell_y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	
	return true;
}

void RandomWalk::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("RandomWalk - %s - Got the map!", __FUNCTION__);
	map_ = *msg;
	
	if(map_known_cells_ == 0)
	{
		for(int i=0 ; i<map_.info.width ; i++)
		{
			for(int j=0 ; j<map_.info.height ; j++)
			{
				if(map_.data[j*map_.info.width+i] == 0) map_known_cells_++;
			}
		}
	}
	
	setGoal();
}

void RandomWalk::odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg)
{
	ROS_DEBUG("RandomWalk - %s - Got an odom msg!", __FUNCTION__);

	geometry_msgs::PoseStamped odom;
	odom.header.frame_id = msg->header.frame_id;
	odom.header.stamp = msg->header.stamp;
	odom.pose.position.x = msg->pose.pose.position.x;	
	odom.pose.position.y = msg->pose.pose.position.y;
	odom.pose.orientation = msg->pose.pose.orientation;
	
	try 
	{
		// Transform the odom into a pose in the map frame
		tf_.transformPose("map", odom, robot_pose_);
	}
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("RandomWalk - %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
}

void RandomWalk::obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("RandomWalk - %s - Got an obstacles msg!", __FUNCTION__);
	obstacles_ = *msg;
}

void RandomWalk::inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("RandomWalk - %s - Got an inflated obstacles msg!", __FUNCTION__);
	inflated_ = *msg;
}

void RandomWalk::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	ROS_INFO("RandomWalk - %s - Already visited %lf%% of the map.", __FUNCTION__, (cells_.cell_height*cells_.cell_width*cells_.cells.size())/(map_.info.resolution*map_.info.resolution*map_known_cells_));

	//if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) ;	
	//if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) ;
	setGoal();
}

void RandomWalk::cellsCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("RandomWalk - %s - Got a visited cells msg!", __FUNCTION__);
	cells_ = *msg;
}

void RandomWalk::goalActiveCallback()
{
	
}

void RandomWalk::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "random_walk_node");
  	
  	ROS_INFO("RandomWalk for ROS v1.0");
	
	srand(time(NULL));

	RandomWalk rw;
	ros::spin();
	
 	return(0);
}

// EOF

