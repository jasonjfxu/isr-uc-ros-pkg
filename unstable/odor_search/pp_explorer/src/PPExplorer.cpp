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
#include "PPExplorer.h"

#include <cstdlib>
#include <math.h>

particle_plume::PPExplorer::PPExplorer() : n_(), pn_("~")
{
	// Get the private parameters...
	pn_.param<std::string>("global_frame_id", global_frame_id_, "map");

	pn_.param("max_visited_cells_coef", max_visited_cells_coef_, 0.35);
	if(max_visited_cells_coef_ < 0.0)
	{
		ROS_WARN("PPExplorer -- You set max_visited_cells_coef to %lf, yet the minimum value is 0, setting max_visited_cells_coef to 0...", max_visited_cells_coef_);
		max_visited_cells_coef_ = 0.0;
	}
	if(max_visited_cells_coef_ > 1.0)
	{
		ROS_WARN("PPExplorer -- You set max_visited_cells_coef to %lf, yet the maximum value is 1, setting max_visited_cells_coef to 1...", max_visited_cells_coef_);
		max_visited_cells_coef_ = 1;
	}
	
	pn_.param("max_odor_visited_cells_coef", max_odor_visited_cells_coef_, 0.50);
	if(max_odor_visited_cells_coef_ < 0.0)
	{
		ROS_WARN("PPExplorer -- You set max_odor_visited_cells_coef to %lf, yet the minimum value is 0, setting max_odor_visited_cells_coef to 0...", max_odor_visited_cells_coef_);
		max_odor_visited_cells_coef_ = 0.0;
	}
	if(max_odor_visited_cells_coef_ > 1.0)
	{
		ROS_WARN("PPExplorer -- You set max_odor_visited_cells_coef to %lf, yet the maximum value is 1, setting max_odor_visited_cells_coef to 1...", max_odor_visited_cells_coef_);
		max_odor_visited_cells_coef_ = 1;
	}
	
	pn_.param("pie_radius", pie_radius_, 0.60);
	if(pie_radius_ < 0) pie_radius_ = 0.0;
	
	pn_.param("pie_slices", pie_slices_, 16);
	if(pie_slices_ < 4)
	{
		ROS_WARN("PPExplorer -- You set pie_slices to %d, yet the minimum value is 4, setting pie_slices to 4...", pie_slices_);
		pie_slices_ = 4;
	}
	if(pie_slices_ > 36)
	{
		ROS_WARN("PPExplorer -- You set pie_slices to %d, yet the maximum value is 36, setting pie_slices to 36...", pie_slices_);
		pie_slices_ = 36;
	}
	
	pn_.param("find_clearing_sim_steps", find_clearing_sim_steps_, 3);
	if(find_clearing_sim_steps_ < 1)
	{
		ROS_WARN("PPExplorer -- You set find_clearing_sim_steps to %d, yet the minimum value is 1, setting find_clearing_sim_steps to 1...", find_clearing_sim_steps_);
		find_clearing_sim_steps_ = 1;
	}
	
	pn_.param("visited_cells_weight", visited_cells_weight_, 1.00);
	pn_.param("current_heading_weight", current_heading_weight_, 0.01);
	pn_.param("odor_weight", odor_weight_, 0.10);
	pn_.param("other_robots_weight", other_robots_weight_, 1.00);
	
	pn_.param("bubble_radius", bubble_radius_, 0.10);
	pn_.param("min_particle_count_difference", min_particle_count_difference_, 5);
	if(min_particle_count_difference_ < 1)
	{
		ROS_WARN("PPExplorer -- You set min_particle_count_difference to %d, yet the minimum value is 1, setting min_particle_count_difference to 1...", min_particle_count_difference_);
		min_particle_count_difference_ = 1;
	}
	
	multi_robot_ = pn_.getParam("unique_id", unique_id_);
	
	pn_.param("debug_slices", debug_slices_, false);
	
	n_.param("move_base/local_costmap/inflation_radius", inflation_radius_, 0.20);
	
	// Subscribe to topics
	odom_sub_.subscribe(n_, "odom", 10);
	tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, global_frame_id_, 10);
    tf_filter_->registerCallback( boost::bind(&PPExplorer::odomCallback, this, _1) );
    
	map_sub_ = n_.subscribe("/map", 1, &PPExplorer::mapCallback, this);
	map_meta_sub_ = n_.subscribe("/map_metadata", 1, &PPExplorer::mapMetaCallback, this);
	cells_sub_ = n_.subscribe("/visited_cells", 1, &PPExplorer::cellsCallback, this);
	plume_sub_ = n_.subscribe("/plume", 1, &PPExplorer::ppCallback, this);
	//plume_sub_.subscribe(n_, "plume", 1, boost::bind(&PPExplorer::ppCallback, this, _1));
	obstacles_sub_ = n_.subscribe("move_base/local_costmap/obstacles", 1, &PPExplorer::obstaclesCallback, this);
	inflated_sub_ = n_.subscribe("move_base/local_costmap/inflated_obstacles", 1, &PPExplorer::inflatedCallback, this);
	
	// Multi-robot stuff
	if(multi_robot_)
	{
		ROS_INFO("PPExplorer -- Going multi-robot, my unique id is %s", unique_id_.c_str());
		
		finish_on_next_recovery_ = false;
		
		recovery_sub_ = n_.subscribe("/pp_explorer/recovery_cells", 100, &PPExplorer::recoveryCellsCallback, this);
		recovery_pub_ = n_.advertise<pp_explorer::GridCellsIdentified>("/pp_explorer/recovery_cells", 10, true);
		
		chatter_sub_ = n_.subscribe("/pp_explorer/chatter", 100, &PPExplorer::chatterCallback, this);
		chatter_pub_ = n_.advertise<pp_explorer::StringIdentified>("/pp_explorer/chatter", 10, true);
		
		poses_sub_ = n_.subscribe("/pp_explorer/poses", 100, &PPExplorer::posesCallback, this);
		poses_pub_ = n_.advertise<pp_explorer::PoseStampedIdentified>("/pp_explorer/poses", 10, true);
	}
	
	debug_pub_ = n_.advertise<nav_msgs::GridCells>("debug_cells", 1, true);
	debug_goal_pub_ = n_.advertise<visualization_msgs::Marker>("pp_goal", 1);
	
	got_map_ = false;
	got_odom_ = false;	
}

particle_plume::PPExplorer::~PPExplorer()
{
	// Do destruction stuff here!
}

bool particle_plume::PPExplorer::isReady()
{
	return (got_map_ && got_odom_);
}


void particle_plume::PPExplorer::ppCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got a plume msg!", __FUNCTION__);
	fromROSMsg(*msg, plume_);
}

void particle_plume::PPExplorer::cellsCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got a visited cells msg!", __FUNCTION__);
	cells_ = *msg;
}

void particle_plume::PPExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got the map!", __FUNCTION__);
	map_ = *msg;
	got_map_ = true;
}

void particle_plume::PPExplorer::mapMetaCallback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got a map metadata msg!", __FUNCTION__);
	map_metadata_ = *msg;
}

void particle_plume::PPExplorer::odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got an odom msg!", __FUNCTION__);

	geometry_msgs::PoseStamped odom;
	odom.header.frame_id = msg->header.frame_id;
	odom.header.stamp = msg->header.stamp;
	odom.pose.position.x = msg->pose.pose.position.x;	
	odom.pose.position.y = msg->pose.pose.position.y;
	odom.pose.orientation = msg->pose.pose.orientation;
	
	try 
	{
		// Transform the odom into a pose in the map frame
		tf_.transformPose(global_frame_id_, odom, robot_pose_);
	}
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("PPExplorer - %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
	
	/*pp_explorer::PoseStampedIdentified my_pose;
	my_pose.sender = unique_id_;
	my_pose.pose = robot_pose_;
	poses_pub_.publish(my_pose);*/
	
	got_odom_ = true;
}

void particle_plume::PPExplorer::obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got an obstacles msg!", __FUNCTION__);
	obstacles_ = *msg;
}

void particle_plume::PPExplorer::inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	ROS_DEBUG("PPExplorer - %s - Got an inflated obstacles msg!", __FUNCTION__);
	inflated_ = *msg;
}

void particle_plume::PPExplorer::recoveryCellsCallback(const pp_explorer::GridCellsIdentified::ConstPtr& msg)
{
	if(msg->sender.compare(unique_id_) == 0) return;
	
	ROS_DEBUG("PPExplorer - %s - Got a recovery cells msg from %s with %d cells.", __FUNCTION__, msg->sender.c_str(), msg->cells.cells.size());
	
	if(recovery_cells_.size() == 0)
	{
		recovery_cells_.push_back(*msg);
	}
	else
	{
		std::vector<pp_explorer::GridCellsIdentified>::iterator cell;
		for(cell = recovery_cells_.begin() ; cell != recovery_cells_.end() ; cell++)
		{
			if(msg->sender.compare(cell->sender) == 0)
			{
				if(msg->cells.cells.size() > 0)
				{
					cell->cells = msg->cells;
				}
				else
				{
					recovery_cells_.erase(cell);
				}
				break;
			}
			else if(cell == recovery_cells_.end() && msg->cells.cells.size() > 0)
			{
				recovery_cells_.push_back(*msg);
			}
		}
	}
}

void particle_plume::PPExplorer::chatterCallback(const pp_explorer::StringIdentified::ConstPtr& msg)
{
	if(msg->sender.compare(unique_id_) == 0) return;

	ROS_INFO("PPExplorer - %s - Got a chatter msg from %s with the following: \"%s\"", __FUNCTION__, msg->sender.c_str(), msg->string.data.c_str());
	
	if(msg->string.data.compare("Exploration complete")==0) finish_on_next_recovery_ = true;
}

void particle_plume::PPExplorer::posesCallback(const pp_explorer::PoseStampedIdentified::ConstPtr& msg)
{
	if(msg->sender.compare(unique_id_) == 0) return;
	
	ROS_DEBUG("PPExplorer - %s - Got a pose msg from %s.", __FUNCTION__, msg->sender.c_str());
	
	if(other_robots_poses_.size() == 0)
	{
		other_robots_poses_.push_back(*msg);
	}
	else
	{
		std::vector<pp_explorer::PoseStampedIdentified>::iterator pose;
		for(pose = other_robots_poses_.begin() ; pose != other_robots_poses_.end() ; pose++)
		{
			if(msg->sender.compare(pose->sender) == 0)
			{
				pose->pose = msg->pose;
				break;
			}
			else if(pose == other_robots_poses_.end())
			{
				other_robots_poses_.push_back(*msg);
			}
		}
	}	
}

bool particle_plume::PPExplorer::findYumiestSlice(geometry_msgs::PoseStamped * goal)
{
	return yumiestSlice(goal, &robot_pose_);
}

bool particle_plume::PPExplorer::yumiestSlice(geometry_msgs::PoseStamped * goal, geometry_msgs::PoseStamped * robot_pose)
{
	ROS_DEBUG("PPExplorer - %s - Yum yum!", __FUNCTION__);	
	
	nav_msgs::GridCells debug_cells;
	
	if(debug_slices_)
	{
		debug_cells.header.frame_id = global_frame_id_;
		debug_cells.cell_width = map_.info.resolution;
		debug_cells.cell_height = map_.info.resolution;
	}

	int robot_cell_x = (int)( robot_pose->pose.position.x / map_.info.resolution );
	int robot_cell_y = (int)( robot_pose->pose.position.y / map_.info.resolution );
	
	ROS_DEBUG("PPExplorer - %s - Map %dx%d @%lf   Robot is at %d %d", __FUNCTION__, map_.info.width, map_.info.height, map_.info.resolution, robot_cell_x, robot_cell_y);
	
	// If our robot is out of the map... not good, not good.
	if(!isInsideTheMap(robot_cell_x, robot_cell_y))
	{
		ROS_ERROR("PPExplorer - %s - Robot is out of bounds!", __FUNCTION__);
		return false;
	}
	
	// Empty the plate with the best slices :D
	best_slices_.clear();
	
	// Calculate the slice area
	double slice_area = (M_PI*pie_radius_*pie_radius_)/pie_slices_; 	//m^2
	// And the angle for half a slice
	double half_slice = 2*M_PI/(2*pie_slices_); // rad
	
	ROS_DEBUG("PPExplorer - %s - Half slice is %lf", __FUNCTION__, half_slice);
	
	int slice_cells;
	int visited_cells_count;
	bool skip_slice;
	int cell_x, cell_y;
	double pos_x, pos_y;
	float slice_goal_x, slice_goal_y;
	double sigma;
	
	double odor_yaw = 0.0;
	double odor_weight = 0.0;
	
	geometry_msgs::PointStamped max_odor;
	max_odor.header.frame_id = global_frame_id_;
	geometry_msgs::PointStamped min_odor;
	min_odor.header.frame_id = global_frame_id_;
	
	int max_particle_count = 0;
	int min_particle_count = plume_.points.size();

	// 1. Check if there is odor present in the pie or not
	bool odor_found = false;
	if(max_visited_cells_coef_ > 0.0)
	{
		BOOST_FOREACH(const pcl::PointXYZI& pt, plume_.points)
		{
			if(isInsidePieRadius(pt.x, pt.y, robot_pose))
			{
				int particle_count = 0;
				BOOST_FOREACH(const pcl::PointXYZI& pti, plume_.points)
				{
					if(sqrt((pt.x-pti.x)*(pt.x-pti.x) + (pt.y-pti.y)*(pt.y-pti.y) + (pt.z-pti.z)*(pt.z-pti.z)) <= bubble_radius_)
					{
						particle_count++;
					}
				}
				if(particle_count > max_particle_count)
				{
					max_particle_count = particle_count;
					max_odor.point.x = pt.x;
					max_odor.point.y = pt.y;
					max_odor.point.z = pt.z;
				}
				if(particle_count < min_particle_count)
				{
					min_particle_count = particle_count;
					min_odor.point.x = pt.x;
					min_odor.point.y = pt.y;
					min_odor.point.z = pt.z;
				}
				odor_found = true;
			}
		}
	}
	double max_visited_cells;
	
	// If odor was found and our odor gradient provided enought information...
	if(odor_found)
	{
		max_visited_cells = max_odor_visited_cells_coef_;
		
		if(!(max_particle_count == 0 || min_particle_count == plume_.points.size()) && max_particle_count-min_particle_count > min_particle_count_difference_)
		{
			odor_weight = odor_weight_;
			//TODO: Verify if the odor vector resulting from max_odor and min_odor is valid. 
			odor_yaw = atan2(max_odor.point.x-min_odor.point.x, max_odor.point.y-min_odor.point.y);
		}
		else
		{
			odor_weight = 0.0;
		}
	}
	else
	{
		max_visited_cells = max_visited_cells_coef_;
		odor_weight = 0.0;
	}
	
	// For each slice...
	for(int slice=0 ; slice<pie_slices_ ; slice++)
	{	
		skip_slice = false;
		
		// 2. Make a rough estimate of where the slice is in terms of map cells
		int upper_bound_x = robot_cell_x;
		int upper_bound_y = robot_cell_y;
		int lower_bound_x = robot_cell_x;
		int lower_bound_y = robot_cell_y;
		
		double start_angle = angles::normalize_angle(tf::getYaw(robot_pose->pose.orientation) + slice*2*half_slice - half_slice);
		
		slice_goal_x = robot_pose->pose.position.x + pie_radius_*cos(start_angle+half_slice);
		slice_goal_y = robot_pose->pose.position.y + pie_radius_*sin(start_angle+half_slice);
		
		ROS_DEBUG("PPExplorer - %s - Slice %d with angle of %lf", __FUNCTION__, slice+1, start_angle);
		
		for(int i=0 ; i<3 ; i++)
		{
			// Cell indexes
			cell_x = (int)((robot_pose->pose.position.x + pie_radius_*cos(start_angle + i*half_slice)) / map_.info.resolution );
			cell_y = (int)((robot_pose->pose.position.y + pie_radius_*sin(start_angle + i*half_slice)) / map_.info.resolution );
		
			if(cell_x > upper_bound_x) upper_bound_x = cell_x;
			else if(cell_x < lower_bound_x) lower_bound_x = cell_x;
			
			if(cell_y > upper_bound_y) upper_bound_y = cell_y;
			else if(cell_y < lower_bound_y) lower_bound_y = cell_y;
		}
		
		// 3. If slice is out of bounds, discard it!
		if(upper_bound_x > map_.info.width-1 || lower_bound_x < 0 || upper_bound_y > map_.info.height-1 || lower_bound_y < 0)
		{
			skip_slice = true;
		}
		
		// 4. If the slice sends the robot to an obstacle or too close to one (inflated obstacle) discard it!
		if(!skip_slice)
		{
			if(!isViableLocalMapGoal(slice_goal_x, slice_goal_y)) skip_slice = true;
		}
		
		// 5. If there are obstacle or unknown cells in the slice, skip this slice
		if(!skip_slice)
		{
			slice_cells = 0;
			for(int i=lower_bound_x ; i<upper_bound_x ; i++)
			{
				for(int j=lower_bound_y ; j<upper_bound_y ; j++)
				{	
					// Cell in map coordinates
					pos_x = i * map_.info.resolution + map_.info.resolution/2;
					pos_y = j * map_.info.resolution + map_.info.resolution/2;
				
					sigma = atan2(pos_y - robot_pose->pose.position.y, pos_x - robot_pose->pose.position.x);
					if(sigma < 0.0) sigma += 2*M_PI;
					sigma = angles::normalize_angle(sigma - (start_angle < 0.0 ? start_angle+2*M_PI : start_angle));
				
					if(isInsidePieRadius(pos_x, pos_y, robot_pose) && sigma >= 0.0 && sigma <= half_slice*2)
					{
						geometry_msgs::Point point;
						point.x = i * map_.info.resolution;
						point.y = j * map_.info.resolution;
					
						// If slice has an obstacle, unknown cell or too close to either one
						if(!isViableGlobalMapCell(i,j))
						{
							skip_slice = true;
						}
						
						if(debug_slices_)
						{
							debug_cells.cells.push_back(point);
							debug_cells.header.stamp = ros::Time::now();
							debug_pub_.publish(debug_cells);
							
							visualization_msgs::Marker marker;
							marker.header.frame_id = "/map";
							marker.header.stamp = ros::Time::now();
							marker.ns = "pp_goal";
							marker.id = 0;
							marker.type = visualization_msgs::Marker::ARROW;
							marker.action = visualization_msgs::Marker::ADD;
							marker.pose.position.x = slice_goal_x;
							marker.pose.position.y = slice_goal_y;
							marker.pose.position.z = 0.25;
							marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, 0.0);
							marker.scale.x = 0.20;
							marker.scale.y = 0.5;
							marker.scale.z = 0.5;
							marker.color.a = 1.0;
							marker.color.r = 1.0;
							marker.color.g = 0.0;
							marker.color.b = 0.0;
							debug_goal_pub_.publish(marker);
						}
						slice_cells++;
					}
					if(skip_slice) break;
				}
				if(skip_slice) break;
			}
		}
		
		if(!skip_slice)
		{
			// 6. Count the number of visited cells inside the slice
			visited_cells_count = 0;
			std::vector<geometry_msgs::Point>::iterator cell;
			for(cell = cells_.cells.begin() ; cell != cells_.cells.end() ; cell++)
			{
				sigma = atan2(cell->y+cells_.cell_height/2 - robot_pose->pose.position.y, cell->x+cells_.cell_width/2 - robot_pose->pose.position.x);
				if(sigma < 0.0) sigma += 2*M_PI;
				sigma = angles::normalize_angle(sigma - (start_angle < 0.0 ? start_angle+2*M_PI : start_angle));
			
				if(isInsidePieRadius(cell->x, cell->y, robot_pose) && sigma >= 0.0 && sigma <= half_slice*2)
				{
					visited_cells_count++;
				}
			}
			
			double slice_yaw = atan2(slice_goal_y-robot_pose->pose.position.y, slice_goal_x-robot_pose->pose.position.x);
			
			SliceOfPie new_slice;
			new_slice.goal.position.x = slice_goal_x;
			new_slice.goal.position.y = slice_goal_y;
			new_slice.goal.orientation = tf::createQuaternionMsgFromYaw(slice_yaw);
			
			double visited_cells_coef = cells_.cell_width*cells_.cell_height*visited_cells_count/slice_area;
			
			if(visited_cells_coef <= max_visited_cells)
			{
				// Cost function
				new_slice.yuminess = visited_cells_weight_ * (visited_cells_coef) + current_heading_weight_ * (fabs(angles::shortest_angular_distance(slice_yaw, tf::getYaw(robot_pose->pose.orientation)))) + odor_weight * (fabs(angles::shortest_angular_distance(slice_yaw, odor_yaw)));
				
				// Multi-robot cost function
				if(multi_robot_)
				{
					std::vector<pp_explorer::PoseStampedIdentified>::iterator pose_it;
					for(pose_it = other_robots_poses_.begin() ; pose_it != other_robots_poses_.end() ; pose_it++)
					{
						if(pose_it->pose.header.stamp - ros::Time::now() > ros::Duration(0.1))
						{
							ROS_WARN("PPExplorer - %s - Discarding %s pose for being too old...", __FUNCTION__, pose_it->sender.c_str());
						}
						else
						{
							//TODO: Finish this!!!
							double robots_delta_x = pose_it->pose.pose.position.x - robot_pose->pose.position.x;
							double robots_delta_y = pose_it->pose.pose.position.y - robot_pose->pose.position.y;
							double other_robot_yaw = atan2(robots_delta_y, robots_delta_x);
							double distance_between_robots = sqrt(robots_delta_x*robots_delta_x + robots_delta_y*robots_delta_y);
							
							new_slice.yuminess += other_robots_weight_*exp(-1*distance_between_robots)*(fabs(angles::shortest_angular_distance(slice_yaw, other_robot_yaw)));
						}
					}
				}
				
				best_slices_.push_back(new_slice);
			}
			else
			{
				ROS_DEBUG("PPExplorer - %s - Discarding this slice %lf %lf due to high percentage of visited cells.", __FUNCTION__, robot_pose->pose.position.x + (pie_radius_ - 0.20)*cos(start_angle+half_slice), robot_pose->pose.position.y + (pie_radius_ - 0.20)*sin(start_angle+half_slice));
			}
			
			ROS_DEBUG("PPExplorer - %s - Found %d visited cells", __FUNCTION__, visited_cells_count);
		}
		else
		{
			ROS_DEBUG("PPExplorer - %s - Discarding this slice %lf %lf due to presence of obstacles.", __FUNCTION__, robot_pose->pose.position.x + (pie_radius_ - 0.20)*cos(start_angle+half_slice), robot_pose->pose.position.y + (pie_radius_ - 0.20)*sin(start_angle+half_slice));
		}
		
		if(debug_slices_)
		{
			ros::Duration(0.1).sleep();
			//debug_cells.cells.clear();
		}
	}
	
	// Ops... None of the slices are any good!!!
	if(best_slices_.size() == 0)
	{
		ROS_DEBUG("PPExplorer - %s - All the slices were discarded!", __FUNCTION__);
		return false;
	}
	
	// 7. Order the slices by yuminess!
	CompareSlices cmp;
	best_slices_.sort(cmp);
	
	// 8. Set the goal!
	this->setGoal(goal);
	
	ROS_DEBUG("PPExplorer - %s - Setting goal to %lf %lf", __FUNCTION__, goal->pose.position.x, goal->pose.position.y);
	
	if(debug_slices_)
	{
		ros::Duration(1.0).sleep();
		debug_cells.cells.clear();
		debug_pub_.publish(debug_cells);
	}
	
	return true;
}

bool particle_plume::PPExplorer::nextYumiestSlice(geometry_msgs::PoseStamped * goal)
{
	best_slices_.pop_front();
	
	if(best_slices_.size() == 0) return false;
	
	this->setGoal(goal);
	
	return true;
}

bool particle_plume::PPExplorer::findClearing(geometry_msgs::PoseStamped * goal)
{
	if(multi_robot_ && finish_on_next_recovery_)
	{
		explorationComplete();
		return false;
	}

	ROS_DEBUG("PPExplorer - %s - Looking for a clearing to go to...", __FUNCTION__);
	
	pp_explorer::GridCellsIdentified recovery_cells;
	recovery_cells.sender = unique_id_;
	recovery_cells.cells.header.frame_id = global_frame_id_;
	
	nav_msgs::GridCells debug_cells;
	
	if(debug_slices_)
	{
		debug_cells.header.frame_id = global_frame_id_;
		debug_cells.cell_width = map_.info.resolution;
		debug_cells.cell_height = map_.info.resolution;
	}
	
	int robot_cell_x = (int)( robot_pose_.pose.position.x / map_.info.resolution );
	int robot_cell_y = (int)( robot_pose_.pose.position.y / map_.info.resolution );
	
	// If our robot is out of the map... not good, not good.
	if(!isInsideTheMap(robot_cell_x, robot_cell_y))
	{
		ROS_ERROR("PPExplorer - %s - Robot is out of bounds!", __FUNCTION__);
		return false;
	}
	
	std::vector<geometry_msgs::Point> last_cells;
	std::vector<geometry_msgs::Point> current_cells;
	std::vector<geometry_msgs::Point> next_cells;
	
	geometry_msgs::Point cell;
	cell.x = robot_cell_x * map_.info.resolution + map_.info.resolution/2;
	cell.y = robot_cell_y * map_.info.resolution + map_.info.resolution/2;
	current_cells.push_back(cell);
	
	while(true)
	{
		ros::spinOnce();
	
		next_cells.clear();
	
		std::vector<geometry_msgs::Point>::iterator cell_it;
		for(cell_it = current_cells.begin() ; cell_it != current_cells.end() ; cell_it++)
		{
			for(int i=(int)(cell_it->x/map_.info.resolution)-1 ; i<(int)(cell_it->x/map_.info.resolution)+2 ; i++)
			{
				for(int j=(int)(cell_it->y/map_.info.resolution)-1 ; j<(int)(cell_it->y/map_.info.resolution)+2 ; j++)
				{
					geometry_msgs::Point new_cell;
					new_cell.x = i * map_.info.resolution;
					new_cell.y = j * map_.info.resolution;
					
					// 1. If cell is itself skip
					if(new_cell.x==cell_it->x && new_cell.y==cell_it->y) continue;
					
					// 2. If cell is obstacle or unknown, or too close to one, skip
					if(!isViableGlobalMapCell(i,j)) continue;
					
					// 3. If cell is already a next cell skip
					if(testCell(&new_cell, &next_cells)) continue; 
					
					// 4. If cell is a current cell skip
					if(testCell(&new_cell, &current_cells)) continue; 
					
					// 5. If cell is a last cell skip
					if(testCell(&new_cell, &last_cells)) continue;
					
					// Multi-robot cell sharing
					if(multi_robot_)
					{
						bool is_recovery_cell = false;
						std::vector<pp_explorer::GridCellsIdentified>::iterator recovery_it;
						for(recovery_it = recovery_cells_.begin() ; recovery_it != recovery_cells_.end() ; recovery_it++)
						{
							if(testCell(&new_cell, &recovery_it->cells.cells))
							{
								is_recovery_cell = true;
								break;
							}
						}
						if(is_recovery_cell) continue;
					}
					
					// 6. We only test cells that are outside the pie!
					if(!isInsidePieRadius(new_cell.x, new_cell.y, &robot_pose_))
					{
						geometry_msgs::PoseStamped dummy_goal;
						dummy_goal.pose.position.x = new_cell.x + map_.info.resolution/2;
						dummy_goal.pose.position.y = new_cell.y + map_.info.resolution/2;
						dummy_goal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dummy_goal.pose.position.y - robot_pose_.pose.position.y, dummy_goal.pose.position.x - robot_pose_.pose.position.x));
					
						// Turn debuf off
						bool turn_debug_back_on = false;
						if(debug_slices_)
						{
							debug_slices_ = false;
							turn_debug_back_on = true;
						}
						
						// Change the visited cells coefficient...
						double temp_visited_cells_coef = max_visited_cells_coef_;
						max_visited_cells_coef_ = 0.0;
						
						nav_msgs::Path dummy_path;
						bool found_clearing = true;
						for(int t=0 ; t<find_clearing_sim_steps_ ; t++)
						{
							if(!yumiestSlice(&dummy_goal, &dummy_goal))
							{
								found_clearing = false;
								break;
							}
							if(dummy_path.poses.size()==0) dummy_path.poses.push_back(dummy_goal);
							else
							{
								// Don't let it go back and forth between two points
								bool nearby_pose = false;
								std::vector<geometry_msgs::PoseStamped>::iterator pose_it;
								for(pose_it=dummy_path.poses.begin() ; pose_it!=dummy_path.poses.end() ; pose_it++)
								{
									double dx = pose_it->pose.position.x - dummy_goal.pose.position.x;
									double dy = pose_it->pose.position.y - dummy_goal.pose.position.y;
									if(sqrt((dx*dx+dy*dy)) < pie_radius_*0.5)
									{
										nearby_pose = true;
										break;
									}
								}
								if(nearby_pose)
								{
									found_clearing = false;
									break;
								}
								else
								{
									dummy_path.poses.push_back(dummy_goal);
								}
							}
						}
						dummy_path.poses.clear();
						
						// And then put it back!
						max_visited_cells_coef_ = temp_visited_cells_coef;
						
						// Turn debug back on
						if(turn_debug_back_on) debug_slices_ = true;
					
						// Yay we found a suitable goal!
						if(found_clearing)
						{
							goal->header.stamp = ros::Time::now();
							goal->header.frame_id = global_frame_id_;
							goal->pose.position.x = new_cell.x + map_.info.resolution/2;
							goal->pose.position.y = new_cell.y + map_.info.resolution/2;
							goal->pose.orientation = tf::createQuaternionMsgFromYaw(atan2(goal->pose.position.y - robot_pose_.pose.position.y, goal->pose.position.x - robot_pose_.pose.position.x));
							
							if(debug_slices_)
							{
								visualization_msgs::Marker marker;
								marker.header.frame_id = "/map";
								marker.header.stamp = ros::Time::now();
								marker.ns = "pp_goal";
								marker.id = 0;
								marker.type = visualization_msgs::Marker::ARROW;
								marker.action = visualization_msgs::Marker::ADD;
								marker.pose.position.x = goal->pose.position.x;
								marker.pose.position.y = goal->pose.position.y;
								marker.pose.position.z = 0.2;
								marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, 0.0);
								marker.scale.x = 0.19;
								marker.scale.y = 0.5;
								marker.scale.z = 0.5;
								marker.color.a = 1.0;
								marker.color.r = 1.0;
								marker.color.g = 1.0;
								marker.color.b = 0.0;
								debug_goal_pub_.publish(marker);
							}
							
							if(multi_robot_)
							{
								recovery_cells.cells.cells.empty();
								recovery_pub_.publish(recovery_cells);
							}
							return true;
						}
					}
					
					// 7. It was not a match, bag and tag
					next_cells.push_back(new_cell);
				}
			}
		}
		
		if(next_cells.size()==0)
		{
			if(multi_robot_)
			{
				recovery_cells.cells.cells.empty();
				recovery_pub_.publish(recovery_cells);
				
				explorationComplete();
			}
			return false;
		}
		
		if(multi_robot_)
		{
			std::vector<geometry_msgs::Point>::iterator recovery_cell_it;
			for(recovery_cell_it = current_cells.begin() ; recovery_cell_it != current_cells.end() ; recovery_cell_it++) recovery_cells.cells.cells.push_back(*recovery_cell_it);
			
			recovery_cells.cells.header.stamp = ros::Time::now();
			
			recovery_pub_.publish(recovery_cells);
		}
		
		if(debug_slices_)
		{
			std::vector<geometry_msgs::Point>::iterator debug_cell_it;
			for(debug_cell_it = current_cells.begin() ; debug_cell_it != current_cells.end() ; debug_cell_it++) debug_cells.cells.push_back(*debug_cell_it);
			
			debug_pub_.publish(debug_cells);
			ros::Duration(0.1).sleep();
		}
		
		last_cells = current_cells;
		current_cells = next_cells;
	}
}


// ******* Helper functions *******

bool particle_plume::PPExplorer::testCell(geometry_msgs::Point * cell, std::vector<geometry_msgs::Point> * cells)
{
	std::vector<geometry_msgs::Point>::iterator cell_it;
	
	for(cell_it = cells->begin() ; cell_it != cells->end() ; cell_it++)
	{
		if(cell->x == cell_it->x && cell->y == cell_it->y) return true;
	}
	return false;
}

void particle_plume::PPExplorer::setGoal(geometry_msgs::PoseStamped * goal)
{
	goal->header.stamp = ros::Time::now();
	goal->header.frame_id = global_frame_id_;
	goal->pose = best_slices_.front().goal;

	if(debug_slices_)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "pp_goal";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = goal->pose.position.x;
		marker.pose.position.y = goal->pose.position.y;
		marker.pose.position.z = 0.2;
		marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI/2, 0.0);
		marker.scale.x = 0.19;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		debug_goal_pub_.publish(marker);
	}
}

bool particle_plume::PPExplorer::isInsidePieRadius(double x, double y, geometry_msgs::PoseStamped * robot)
{
	return sqrt((x - robot->pose.position.x)*(x - robot->pose.position.x) + (y - robot->pose.position.y)*(y - robot->pose.position.y)) <= pie_radius_;
}

bool particle_plume::PPExplorer::isInsideTheMap(int x, int y)
{
	return (x<=map_.info.width-1 && x>=0 && y<=map_.info.height-1 && y>=0);
}

bool particle_plume::PPExplorer::isViableGlobalMapCell(int x, int y)
{
	if(!isInsideTheMap(x,y)) return false;
	
	if(map_.data[y*map_.info.width+x] != 0) return false;
					
	int cir = ceil(inflation_radius_/map_.info.resolution)+1;
	for(int i=(x-cir<0 ? 0 : x-cir) ; i<(x+cir>map_.info.width ? map_.info.width : x+cir) ; i++)
	{
		for(int j=(y-cir<0 ? 0 : y-cir) ; j<(y+cir>map_.info.height ? map_.info.height : y+cir) ; j++)
		{
			if(map_.data[j*map_.info.width+i] != 0 && sqrt((x-i)*(x-i)+(y-j)*(y-j)) <= (float)cir-1.0)
			{
				return false;
			}
		}
	}
	return true;
}

bool particle_plume::PPExplorer::isViableLocalMapGoal(double x, double y)
{
	std::vector<geometry_msgs::Point>::iterator obstacle;
	
	for(obstacle = obstacles_.cells.begin() ; obstacle != obstacles_.cells.end() ; obstacle++)
	{
		if(x >= obstacle->x && x <= obstacle->x+obstacles_.cell_width && y >= obstacle->y && y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	
	for(obstacle = inflated_.cells.begin() ; obstacle != inflated_.cells.end() ; obstacle++)
	{
		if(x >= obstacle->x && x <= obstacle->x+obstacles_.cell_width && y >= obstacle->y && y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	return true;	
}

double particle_plume::PPExplorer::getExploredArea()
{
	int map_cell_count = 0;

	for(int i=0 ; i<map_.info.width ; i++)
	{
		for(int j=0 ; j<map_.info.height ; j++)
		{
			if(map_.data[j*map_.info.width+i] == 0) map_cell_count++;
		}
	}
	
	return (cells_.cell_height*cells_.cell_width*cells_.cells.size())/(map_.info.resolution*map_.info.resolution*map_cell_count);
}

void particle_plume::PPExplorer::explorationComplete()
{
	pp_explorer::StringIdentified finished_msg;
	finished_msg.sender = unique_id_;
	finished_msg.string.data = "Exploration complete";
	chatter_pub_.publish(finished_msg);
}

// EOF

