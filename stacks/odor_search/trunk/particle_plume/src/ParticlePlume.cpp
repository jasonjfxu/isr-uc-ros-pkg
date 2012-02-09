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
#include "ParticlePlume.h"

#include <cstdlib>
#include <math.h>

particle_plume::ParticlePlume::ParticlePlume() : n_(), pn_("~")
{
	// Get the private parameters...
	pn_.param<std::string>("global_frame_id", global_frame_id_, "map");
	pn_.param("bubble_radius", bubble_radius_, 0.10);
	pn_.param("max_particles_per_bubble", max_particles_per_bubble_, 1000);
	pn_.param("particle_life_time", particle_life_time_, 0);
	pn_.param("publish_frequency", publish_frequency_, 2.0);
	pn_.param("cell_size", cell_size_, 0.05);
	pn_.param<std::string>("sensor_model", sensor_model_, "Figaro 2620");
	
	// Subscribe to the chemical sensor topic
	nose_sub_.subscribe(n_, "nose", 10);
	tf_filter_ = new tf::MessageFilter<lse_sensor_msgs::Nostril>(nose_sub_, tf_, global_frame_id_, 10);
    tf_filter_->registerCallback( boost::bind(&ParticlePlume::noseCallback, this, _1) );
	
	// Advertise the plume as a PointCloud2
	plume_pub_.advertise(n_, "plume", 1);
	
	// Advertise the visited cells grid as a PointCloud2
	cells_pub_ = n_.advertise<nav_msgs::GridCells>("visited_cells", 1, true);
	
	cells_.header.frame_id = global_frame_id_;
	cells_.cell_width = cell_size_;
	cells_.cell_height = cell_size_;
	
	cells_changed_ = false;
}

particle_plume::ParticlePlume::~ParticlePlume()
{
	// Do destruction stuff here!
}

void particle_plume::ParticlePlume::noseCallback(const boost::shared_ptr<const lse_sensor_msgs::Nostril>& msg)
{
	if(msg->sensor_model.compare(sensor_model_)!=0) return;
	
	// Determine the number of particles proportional to the chemical concentrarion
	int number_of_particles = (int)((msg->reading-msg->clean_air) * max_particles_per_bubble_ / (msg->max_reading-msg->clean_air));
	if(number_of_particles<0) number_of_particles=0;
	
	geometry_msgs::PointStamped bubble_center;
	geometry_msgs::PointStamped nose;
	nose.header.frame_id = msg->header.frame_id;
	nose.header.stamp = msg->header.stamp;
	nose.point.x = 0.0;	
	nose.point.y = 0.0;
	nose.point.z = 0.0;
	
	try 
	{
		// Transform the bubble center from the chemical sensor frame to the map frame
		tf_.transformPoint(global_frame_id_, nose, bubble_center);
	}
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("ParticlePlume -- Error: %s", ex.what());
		return;
	}
	
	// If we have some particles to add...
	if(number_of_particles > 0)
	{
		// Store the new data in the buffer 
		BubbleData new_odor_reading;
		new_odor_reading.number_of_particles = number_of_particles;
		new_odor_reading.bubble_center = bubble_center;
	
		odor_readings_.push_back(new_odor_reading);
	}
	
	// Update the visited cells data structure
	std::vector<CellCandidate> cell_candidates;
	std::vector<CellCandidate>::iterator cell_candidate;
	
	int start_cell_x = ceil((bubble_center.point.x - bubble_radius_) / cell_size_);
	int end_cell_x = ceil((bubble_center.point.x + bubble_radius_) / cell_size_);
	int start_cell_y = ceil((bubble_center.point.y - bubble_radius_) / cell_size_);
	int end_cell_y = ceil((bubble_center.point.y + bubble_radius_) / cell_size_);
	
	for(int x=start_cell_x ; x<end_cell_x ; x++)
	{
		for(int y=start_cell_y ; y<end_cell_y ; y++)
		{
			CellCandidate new_cell;
			new_cell.point.x = x*cell_size_;
			new_cell.point.y = y*cell_size_;
			new_cell.point.z = 0.0;
			new_cell.push = true;
			
			cell_candidates.push_back(new_cell);
		}
	}
	
	std::vector<geometry_msgs::Point>::iterator cell;
	for(int i=0 ; i<cells_.cells.size() ; i++)
	{
		cell = cells_.cells.begin()+i;
		for(cell_candidate = cell_candidates.begin() ; cell_candidate != cell_candidates.end() ; cell_candidate++)
		{
			if(cell_candidate->point.x == cell->x && cell_candidate->point.y == cell->y)
			{
				cell_candidate->push = false;
				cells_birth_[i] = ros::Time::now();
			}
		}
	}
	
	for(cell_candidate = cell_candidates.begin() ; cell_candidate != cell_candidates.end() ; cell_candidate++)
	{
		if(cell_candidate->push == true)
		{
			cells_.cells.push_back(cell_candidate->point);
			cells_birth_.push_back(ros::Time::now());
			cells_changed_ = true;
		}
	}
}

void particle_plume::ParticlePlume::publishPlume()
{
	ros::Rate r(publish_frequency_);
	
	std::vector<BubbleData>::iterator reading;
	std::vector<BubbleData>::iterator bubble_it;
	
	bool push;
	bool publish_plume;
	
	// As long as we're good to go...
	while(n_.ok())
	{	
		publish_plume = false;
		
		// If we have new readings to consider...
		if(odor_readings_.size() > 0)
		{
			pcl::PointCloud<Particle> new_plume;
		
			for(reading=odor_readings_.begin() ; reading!=odor_readings_.end() ; reading++)
			{
				// Generate the cloud of particles for the current odor reading
				for(int i=0 ; i<reading->number_of_particles ; i++)
				{
					double r = drand()*bubble_radius_;
					double theta = drand()*M_PI;
					double phi = drand()*2*M_PI;
					
					Particle particle;
					particle.x = reading->bubble_center.point.x + r*sin(theta)*cos(phi);
					particle.y = reading->bubble_center.point.y + r*sin(theta)*sin(phi);
					particle.z = reading->bubble_center.point.z + r*cos(theta);
					particle.intensity = 100.0;
					
					// Now lets check if we can add the newly created particle to the cloud
					push = true;
					for(bubble_it=odor_readings_.begin() ; bubble_it!=reading ; bubble_it++)
					{
						double dx = bubble_it->bubble_center.point.x - particle.x;
						double dy = bubble_it->bubble_center.point.y - particle.y;
						double dz = bubble_it->bubble_center.point.z - particle.z;
						// If the particle is inside a newer bubble, dont add it
						if(sqrt(dx*dx + dy*dy + dz*dz) <= bubble_radius_)
						{
							push = false;
							break;
						}
					}
					if(push == true) new_plume.points.push_back(particle);
				}
			}
			
			// Now add the previous iteration points on plume_ to the new cloud on new_plume
			for(int i=0 ; i<plume_.width ; i++)
			{
				Particle particle;
				particle.x = plume_.points[i].x;
				particle.y = plume_.points[i].y;
				particle.z = plume_.points[i].z;
				particle.intensity = plume_.points[i].intensity;
				
				// Decrease the particle intensity if needed
				if(particle_life_time_ > 0 && particle.intensity > 0.0)
				{
					particle.intensity -= 100.0/publish_frequency_/(particle_life_time_*60.0);
					if(particle.intensity < 0.0) particle.intensity = 0.0;
				}
				
				// Now lets check if this particle still belongs on the clowd
				push = true;
				for(reading=odor_readings_.begin() ; reading!=odor_readings_.end() ; reading++)
				{		
					double dx = reading->bubble_center.point.x - particle.x;
					double dy = reading->bubble_center.point.y - particle.y;
					double dz = reading->bubble_center.point.z - particle.z;
					// If the particle is inside a newer bubble, dont add it
					if(sqrt(dx*dx + dy*dy + dz*dz) <= bubble_radius_)
					{
						push=false;
						break;
					}
				}
				if(push==true && particle.intensity > 0.0) new_plume.points.push_back(particle);
			}
			
			// Clear the readings buffer
			odor_readings_.clear();
		
			// Store the new plume and publish it...
			plume_ = new_plume;
			plume_.width = plume_.points.size();
			plume_.height = 1;
			
			publish_plume = true;
		}
		// If we dont have new readings and we have a particle life time defined
		else if(particle_life_time_ > 0)
		{
			pcl::PointCloud<Particle> new_plume;
			for(int i=0 ; i<plume_.width ; i++)
			{
				Particle particle;
				particle.x = plume_.points[i].x;
				particle.y = plume_.points[i].y;
				particle.z = plume_.points[i].z;
				particle.intensity = plume_.points[i].intensity;
				
				if(particle.intensity > 0.0)
				{
					particle.intensity -= 100.0/publish_frequency_/(particle_life_time_*60.0);
					if(particle.intensity < 0.0) particle.intensity = 0.0;
				}
				
				if(particle.intensity > 0.0) new_plume.points.push_back(particle);
			}
			// Store the new plume and publish it...
			plume_ = new_plume;
			plume_.width = plume_.points.size();
			plume_.height = 1;
		}
		
		//TODO: Delete old cells!
		if(particle_life_time_ > 0)
		{
			std::vector<int> cells_to_delete;
			for(int i=0 ; i<cells_birth_.size() ; i++)
			{
				if((ros::Time::now() - cells_birth_[i]).toSec() > ros::Duration(particle_life_time_*60.0).toSec()) cells_to_delete.push_back(i);
			}
		
			for(int i=cells_to_delete.size()-1 ; i>=0 ; i--)
			{
				cells_.cells.erase(cells_.cells.begin()+cells_to_delete[i]);
				cells_birth_.erase(cells_birth_.begin()+cells_to_delete[i]);
			}
		}
		
		//if(publish_plume)
		//{
			plume_.header.stamp = ros::Time::now();
			plume_.header.frame_id = global_frame_id_;	
			plume_pub_.publish(plume_);
		//}
		
		//if(cells_changed_)
		//{
			cells_.header.stamp = ros::Time::now();
			cells_pub_.publish(cells_);
			cells_changed_ = false;
		//}
		
		// Spin...
		ros::spinOnce();
		// ...and sleep!
		r.sleep();
	}
}

float particle_plume::ParticlePlume::drand()
{
	return ((rand()+1.0)/(RAND_MAX+1.0));
}

double particle_plume::ParticlePlume::randomNormal()
{
	return (sqrt(-2*log(drand()))*cos(2*M_PI*drand()));
}

// EOF

