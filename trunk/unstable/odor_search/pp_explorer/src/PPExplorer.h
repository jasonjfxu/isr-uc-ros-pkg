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
//#include <pcl_ros/subscriber.h>
#include <angles/angles.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <list>

#include <pp_explorer/GridCellsIdentified.h>
#include <pp_explorer/StringIdentified.h>
#include <pp_explorer/PoseStampedIdentified.h>

namespace particle_plume
{
	/*! \class PPExplorer PPExplorer.h "PPExplorer.h"
	 *  \brief Particle plume C++ class.
	 *
	 * Short explanation...
	 */
	class PPExplorer
	{
		public:
		//! Constructor.
		PPExplorer();
		//! Destructor
		~PPExplorer();
		
		//! Calculate the yumiest slice.
		/*!
		 *  This function determines which direction the robot should take and sets a new goal.
		 *
		 *  \param goal    	Robot goal pose.
		 *
		 *  \return 		True if succeeded in setting a goal, false otherwise.
		 */
		bool findYumiestSlice(geometry_msgs::PoseStamped * goal);
		
		//! Get the nest yumiest slice.
		/*!
		 *  This function pops the first slice in store and sets the next best heading as the robot goal.
		 *  This is called when the last goal failed.
		 *
		 *  \param goal    	Robot goal pose.
		 *
		 *  \return 		True if succeeded in setting a goal, false otherwise.
		 */
		bool nextYumiestSlice(geometry_msgs::PoseStamped * goal);
		
		//! Find an unexplored clearing.
		/*!
		 *  This function scans the map for an unexplored clearing and sets the goal to the middle
		 *  of the first clearing that is found.
		 *
		 *  \param goal    	Robot goal pose.
		 *
		 *  \return 		True if succeeded in setting a goal, false otherwise.
		 */
		bool findClearing(geometry_msgs::PoseStamped * goal);
		
		//! Get explored area.
		/*!
		 *  Get the percentage of explored area.
		 *
		 *  \return 		The percentage of explored area.
		 */
		double getExploredArea();
		
		//! If PPExplorer is ready to start or not.
		bool isReady();
	
		private:
	
		//! Node handler.
		ros::NodeHandle n_;
		//! Private node handler.
		ros::NodeHandle pn_;
		//! Odometry topic subscriber.
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
		tf::TransformListener tf_;
  		tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
		//! Map topic subscriber.
		ros::Subscriber map_sub_;
		//! Map metadata topic subscriber.
		ros::Subscriber map_meta_sub_;
		//! Visited cells topic subscriber.
		ros::Subscriber cells_sub_;
		//! Particle plume topic publisher.
		ros::Subscriber plume_sub_;
		//pcl_ros::Subscriber<pcl::PointXYZI> plume_sub_;
		//! Obstacles topic subscriber.
		ros::Subscriber obstacles_sub_;
		//! Inflated Obstacles topic subscriber.
		ros::Subscriber inflated_sub_;
		
		// ******* Multi-robot stuff *******
		//! If PPExplorer is working in multi robot or not
		bool multi_robot_;
		//! Unique robot id.
		std::string unique_id_;
		//! Recovery cells sub
		ros::Subscriber recovery_sub_;
		//! Recovery cells pub
		ros::Publisher recovery_pub_;
		//! Chatter sub
		ros::Subscriber chatter_sub_;
		//! Chatter pub
		ros::Publisher chatter_pub_;
		//! Pose sub
		ros::Subscriber poses_sub_;
		//! Pose pub
		ros::Publisher poses_pub_;
		
		//! Data structure that holds the recovery cells.
		std::vector<pp_explorer::GridCellsIdentified> recovery_cells_;
		
		//! Other robots poses on the global frame referential
		std::vector<pp_explorer::PoseStampedIdentified> other_robots_poses_;
		
		//! If any robot finishes it tells the other robots to stop on the next recovery behavior.
		bool finish_on_next_recovery_;
		// *********************************
		
		// ******* Debug stuff *******
		//! Debug cells pub
		ros::Publisher debug_pub_;
		//! Debug goal pub
		ros::Publisher debug_goal_pub_;
		
		bool debug_slices_;
		// ***************************
		
		//! Particle plume data structure.
		pcl::PointCloud<pcl::PointXYZI> plume_;
		
		//! Data structure that holds the visited cells.
		nav_msgs::GridCells cells_;
		
		//! Map
		nav_msgs::OccupancyGrid map_;
		//! Map metadata
		nav_msgs::MapMetaData map_metadata_;
		
		//! Data structure that holds the obstacle cells.
		nav_msgs::GridCells obstacles_;
		//! Data structure that holds the inflated obstacle cells.
		nav_msgs::GridCells inflated_;
		
		//! Robot pose on the global frame referential
		geometry_msgs::PoseStamped robot_pose_;
		
		//! Global frame_id.
		std::string global_frame_id_;
		
		//! pie radius.
		/*!
		 * The pie radius determines the radius around the robot for which viable paths will be calculated.
		 */
		double pie_radius_;
		//! pie slices.
		/*!
		 * The number of pie slices determines the number of paths that will be considered.
		 */
		int pie_slices_;
		//! Visited cells coefficient.
		/*!
		 * This coefficient gives the weight given to the number of visited cells.
		 */
		double max_visited_cells_coef_;
		//! Visited cells coefficient.
		/*!
		 * This coefficient gives the weight given to the number of visited cells during when odor is present, should be higher than max_visited_cells_coef.
		 *
		 * \sa max_visited_cells_coef_
		 */
		double max_odor_visited_cells_coef_;
		//! Number of simulations.
		/*!
		 * Number of simulations performed during the find clearing stage.
		 */
		int find_clearing_sim_steps_;
		//! Inflation radius.
		/*!
		 * The inflation radius used by the navigation stack.
		 */
		double inflation_radius_;
		
		//! Visited cells weight
		/*!
		 * The weight given to the ammount of visited cells in the cost function.
		 */
		double visited_cells_weight_;
		//! Current heading weight
		/*!
		 * The weight given to the robot's current heading in the cost function.
		 */
		double current_heading_weight_;
		//! Odor weight
		/*!
		 * The weight given to the odor presence in the cost function.
		 */
		double odor_weight_;
		//! Odor weight
		/*!
		 * The weight given to the heading in regard to the other robots in the cost function.
		 */
		double other_robots_weight_;
		
		//! Bubble radius
		/*!
		 * The radius of the bubble used to count particles during odor gradient extraction
		 */
		double bubble_radius_;
		//! Minimum particle count difference
		/*!
		 * The minimum particle count difference allowed for an odor gradient to be considered.
		 */
		int min_particle_count_difference_;
		
		//! Data structure for storing slices of pie, yum!
		struct SliceOfPie
		{
			geometry_msgs::Pose goal;
			double yuminess;
		};
		
		//! List of the current best slices of pie
		std::list<SliceOfPie> best_slices_;
		
		//! Slice of pie comparing class.
		 class CompareSlices
		 {
  			public:
    		bool operator()(const SliceOfPie & first, const SliceOfPie & second)
    		{
      			if(first.yuminess < second.yuminess) return true;
				else return false;	
    		}
		};
		
		//! Particle plume callback.
		/*!
		 *  This function receives messages from the plume topic.
		 *
		 *  \param msg    Incoming pcl::PointCloud<pcl::PointXYZI> msg.
		 */
		void ppCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	
		//! Visited cells topic callback.
		/*!
		 *  This function receives messages from the visited cells topic.
		 *
		 *  \param msg    Incoming nav_msgs::GridCells msg.
		 */
		void cellsCallback(const nav_msgs::GridCells::ConstPtr& msg);
		
		//! Map topic callback.
		/*!
		 *  This function receives messages from the map topic.
		 *
		 *  \param msg    Incoming nav_msgs::OccupancyGrid map msg.
		 */
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		
		//! Map metadata topic callback.
		/*!
		 *  This function receives messages from the map_metadata topic.
		 *
		 *  \param msg    Incoming nav_msgs::MapMetaData msg.
		 */
		void mapMetaCallback(const nav_msgs::MapMetaData::ConstPtr& msg);
		
		//! Odometry topic callback.
		/*!
		 *  This function receives messages from the odom topic.
		 *
		 *  \param msg    Incoming nav_msgs::Odometry msg.
		 */
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
		
		//! Obstacles topic callback.
		/*!
		 *  This function receives messages from the obstacles topic.
		 *
		 *  \param msg    Incoming nav_msgs::GridCells msg.
		 */
		void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg);
		
		//! Inflated obstacles topic callback.
		/*!
		 *  This function receives messages from the inflated obstacles topic.
		 *
		 *  \param msg    Incoming nav_msgs::GridCells msg.
		 */
		void inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg);
		
		//! Recovery cells topic callback.
		/*!
		 *  This function receives messages from the recovery cells topic.
		 *
		 *  \param msg    Incoming pp_explorer::GridCellsIdentified msg.
		 */
		void recoveryCellsCallback(const pp_explorer::GridCellsIdentified::ConstPtr& msg);
		
		//! Chatter topic callback.
		/*!
		 *  This function receives messages from the chatter topic for multi-robot communication.
		 *
		 *  \param msg    Incoming pp_explorer::StringIdentified msg.
		 */
		void chatterCallback(const pp_explorer::StringIdentified::ConstPtr& msg);
		
		//! Poses topic callback.
		/*!
		 *  This function receives messages from the poses topic for multi-robot communication.
		 *
		 *  \param msg    Incoming pp_explorer::PoseStampedIdentified msg.
		 */
		void posesCallback(const pp_explorer::PoseStampedIdentified::ConstPtr& msg);
		
		
		//! Set goal.
		/*!
		 *  This function sets a goal to the robot from the current yumiest slice.
		 *
		 *  \param goal    Robot goal pose.
		 */
		void setGoal(geometry_msgs::PoseStamped * goal);
		
		//! Test cell
		/*!
		 *  This is a helper function for findClearing and it test for a cell inside a vector of cells.
		 *
		 *  \param cell     The cell to be tested.
		 *  \param cells	The vector of cells to be tested.
		 *
		 *  \return 		True if the cell is found on the vector cells, false otherwise.
		 */
		bool testCell(geometry_msgs::Point * cell, std::vector<geometry_msgs::Point> * cells);
		
		//! Calculate the yumiest slice.
		/*!
		 *  This function determines which direction the robot should take and sets a new goal.
		 *
		 *  \param goal    		Robot goal pose.
		 *  \param robot_pose   Robot initial pose.
		 *
		 *  \return 			True if succeeded in setting a goal, false otherwise.
		 */
		bool yumiestSlice(geometry_msgs::PoseStamped * goal, geometry_msgs::PoseStamped * robot_pose);
		
		//! Determine if point is inside the pie radius.
		/*!
		 *  This helper function determines if a x y point is inside the pie radius centered at the robot pose or not.
		 *
		 *  \param x    		Test point x coordinate.
		 *  \param y    		Test point y coordinate.
		 *  \param robot   		Robot pose.
		 *
		 *  \return 			True if point is inside pie radius, false otherwise.
		 */
		bool isInsidePieRadius(double x, double y, geometry_msgs::PoseStamped * robot);
		
		//! Determine if a cell is inside the map.
		/*!
		 *  This helper function determines if a cell is inside the map ir not.
		 *
		 *  \param x    		Cell x.
		 *  \param y    		Cell y.
		 *
		 *  \return 			True if the cell is inside the map, false otherwise.
		 */
		bool isInsideTheMap(int x, int y);
		
		//! Determine if a cell is viable to go to.
		/*!
		 *  This helper function determines if a cell is not an obstacle, not unknown, or too close to either of the previous two.
		 *
		 *  \param x    		Cell x.
		 *  \param y    		Cell y.
		 *
		 *  \return 			True if the cell is viable, false otherwise.
		 */
		bool isViableGlobalMapCell(int x, int y);
		
		//! Determine if a goal is viable to go to.
		/*!
		 *  This helper function determines if a goal is not an obstacle, or inflated obstacle.
		 *
		 *  \param x    		Goal x.
		 *  \param y    		Goal y.
		 *
		 *  \return 			True if the goal is viable, false otherwise.
		 */
		bool isViableLocalMapGoal(double x, double y);
		
		//! Exploration complete
		/*!
		 *  Publish that exploration is complete on the chatter topic.
		 */
		void explorationComplete();
		
		bool got_map_;
		bool got_odom_;
	};
}

// EOF

