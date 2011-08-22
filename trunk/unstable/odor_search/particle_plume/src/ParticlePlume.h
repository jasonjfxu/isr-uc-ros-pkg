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
* Author: Gonçalo Cabrita on 28/11/2010
*********************************************************************/
#include <ros/ros.h>
#include <lse_sensor_msgs/Nostril.h>
#include <pcl_ros/publisher.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/GridCells.h>
#include <vector>

#include "Particle.h"

namespace particle_plume
{
	/*! \class ParticlePlume ParticlePlume.h "ParticlePlume.h"
	 *  \brief Particle plume C++ class.
	 *
	 * This class receives data from electronic noses converting the chemical concentration data into a virtual plume.
	 * This virtual plume is represented by a 3D cloud of particles where a higher concentration of particles indicates a higher chemical concentration.
	 * Each chemical reading is converted into a small cloud of particles centered around the reading point over a normal distribution with a standard deviation of 1.
	 * Chemical data can be gathered from multiple sources, various sensors on a robot, various robots, stationary sensor network nodes, etc.
	 */
	class ParticlePlume
	{
		public:
		//! Constructor.
		ParticlePlume();
		//! Destructor
		~ParticlePlume();
	
		//! Publish plume.
		/*!
		 *  This function publishes the particle plume.
		 */
		void publishPlume();
	
		private:
	
		//! Data coming from the nose topic is stored in this form on a buffer
		struct BubbleData
		{
			//! Center of the bubble in relation to the global frame as well as the time stamp
			geometry_msgs::PointStamped bubble_center;
			//! The number of particles to be spawned
			int number_of_particles;
		};
		
		//! Auxiliary data structure
		struct CellCandidate
		{
			geometry_msgs::Point point;
			bool push;	
		};
	
		//! Node handler.
		ros::NodeHandle n_;
		//! Private node handler.
		ros::NodeHandle pn_;
		
		//! Particle plume topic publisher.
		pcl_ros::Publisher<Particle> plume_pub_;
		//! Visited cells topic publisher.
		ros::Publisher cells_pub_;
		
		//! Nose topic subscriber.
		message_filters::Subscriber<lse_sensor_msgs::Nostril> nose_sub_;
		//! Transform listener.
		tf::TransformListener tf_;
  		tf::MessageFilter<lse_sensor_msgs::Nostril> * tf_filter_;
	
		//! Particle plume data structure.
		pcl::PointCloud<Particle> plume_;
		
		//! Data structure that holds the visited cells.
		nav_msgs::GridCells cells_;
		
		//! Data structure to hold the birth time of the visited cells.
		std::vector<ros::Time> cells_birth_;
		
		//! Flag for wether the cells_ data structure was changed or not
		bool cells_changed_;
	
		//! Buffer for holding pre-processed incoming chemical readings
		std::vector<BubbleData> odor_readings_;
	
		//! Global frame_id.
		std::string global_frame_id_;
	
		//! Bubble radius.
		/*!
		 * The bubble radius determines the sphere inside which points will be generated for each chemical reading.
		 */
		double bubble_radius_;
		//! Maximum number of points per bubble.
		/*!
		 * This value determines the number of points generated inside a sphere for a maximum chemical reading.
		 */
		int max_particles_per_bubble_;
		//! Particle life time.
		/*!
		 * Each particle can have a life time in minutes. Once this time is up the particle is removed from the plume. A value of 0 indicates that the particles are never removed.
		 */
		int particle_life_time_;
		//! Publish frequency.
		/*!
		 * The rate at which the particle plume is published in Hz. Plume update occurs every time a chemical reading arrives.
		 */
		double publish_frequency_;
		//! Cell size.
		/*!
		 * The height and width for each visited cell.
		 */
		double cell_size_;
		//! Sensor Model.
		/*!
		 * The sensor model accepted for this plume.
		 */
		std::string sensor_model_;
	
		//! Nose topic callback.
		/*!
		 *  This function receives messages from the nose topic.
		 *
		 *  \param msg    Incoming lse_sensor_msgs::Nostri msg.
		 */
		void noseCallback(const boost::shared_ptr<const lse_sensor_msgs::Nostril>& msg);
		
		//! Uniform distribution.
		/*!
		 *  Uniform distribution, [0 ... 1]
		 * 
		 *  \return The random value.	
		 */
		float drand();
		//! Normal distribution.
		/*!
		 *  Normal distribution, centered on 0, std dev 1.
		 *
		 *  \return The generated value.
		 */
		double randomNormal();
	};
}

// EOF

