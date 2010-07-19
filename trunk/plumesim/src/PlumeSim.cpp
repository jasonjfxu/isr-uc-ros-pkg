/*
 *  PlumeSim.cpp
 *  
 *
 *  Created by Gonçalo Cabrita and Pedro Sousa on 31/05/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	PlumeSim node for ROS
 *
 */

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*!
 *
 * \file PlumeSim.cpp
 *
 * \brief This file implements the PlumeSim node for generting simulated plumes.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 31/05/2010
 * \version 0.2
 *
 * \bug none discovered
 *
 * \note
 *
 */

#define NODE_VERSION 1.00

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "plumesim/ReadPlumeSim.h"

// Types of sources
#include "utils/PSSources.h"


class PlumeSim
{
	
	public:
	/*!
	 *
	 * \fn PlumeSim()
	 * \brief PlumeSim constructor. This is the class constructor.
	 * \return none.
	 *
	 */
	PlumeSim();
	//! PlumeSim destructor.
    	/*!
	 * This is the class destructor.
     	 */
	~PlumeSim();

	/*!
	 *
	 * \fn bool ReadPlumeSim(plumesim::ReadPlumeSim::Request  &req, plumesim::ReadPlumeSim::Response &res)
	 * \brief Service function for returning chemical values for robots.
	 * \return true if service succeeded.
	 * \ return false otherwise.
	 *
	 */
	bool ReadPlumeSim(plumesim::ReadPlumeSim::Request  &req, plumesim::ReadPlumeSim::Response &res);
	
	/*!
	 *
	 * \fn virtual void UpdatePlume()
	 * \brief Update function for the plume.
	 * \return none.
	 *
	 */
	void UpdatePlume();

	/*!
	 *
	 * \fn virtual double getFrequency()
	 * \brief Returns the frequency of the plume.
	 * \return Frequency.
	 *
	 */
	double getFrequency();
	
	private:
	
	PSSource * source;
	
	ros::NodeHandle n;
	ros::Publisher marker_pub;
	ros::ServiceServer service;

	visualization_msgs::Marker points;
	
	double freq;
	
	double plume_color_r, plume_color_g, plume_color_b, plume_color_a;
	
	std::string plume_ns;
	int plume_id;
};

// *****************************************************************************
// Constructor.
PlumeSim::PlumeSim()
{		
	ROS_INFO("PlumeSim for ROS %.2f", NODE_VERSION);
	
	marker_pub = n.advertise<visualization_msgs::Marker>("plumesim_markers", 10);
	service = n.advertiseService("read_plumesim", &PlumeSim::ReadPlumeSim, this);
	
	// Setting the plume type
	if(!n.hasParam("plumesim/type"))
	{
		ROS_FATAL("No type of simulation was defined!");
		ROS_BREAK();
	}
	std::string type;
	n.getParam("plumesim/type", type);
	
	if(type.compare("meadering") == 0)
	{
		source = new PSMeadering();
		PSMeadering * meandering = dynamic_cast<PSMeadering*>(source);		
		
		n.getParam("plumesim/releasepoint/x", meandering->startPoint.px);
		n.getParam("plumesim/releasepoint/y", meandering->startPoint.py);
		n.getParam("plumesim/releasepoint/z", meandering->startPoint.pz);
		n.getParam("plumesim/arena/start/x", meandering->arenaRect.startPoint.px);
		n.getParam("plumesim/arena/start/y", meandering->arenaRect.startPoint.py);
		n.getParam("plumesim/arena/end/x", meandering->arenaRect.endPoint.px);
		n.getParam("plumesim/arena/end/y", meandering->arenaRect.endPoint.py);
		n.getParam("plumesim/number_of_points", meandering->numOfPlumePoints);
		n.getParam("plumesim/release_rate", meandering->releaseRate);
		n.getParam("plumesim/dispersion_coeficient", meandering->dispersionCoeficient);
		n.getParam("plumesim/radius", meandering->radius);
	}
	else if(type.compare("gaussian") == 0)
	{
		source = new PSGaussian();
		PSGaussian * gaussian = dynamic_cast<PSGaussian*>(source);
		
		n.getParam("plumesim/releasepoint/x", gaussian->startPoint.px);
		n.getParam("plumesim/releasepoint/y", gaussian->startPoint.py);
		n.getParam("plumesim/releasepoint/z", gaussian->startPoint.pz);
		n.getParam("plumesim/arena/start/x", gaussian->arenaRect.startPoint.px);
		n.getParam("plumesim/arena/start/y", gaussian->arenaRect.startPoint.py);
		n.getParam("plumesim/arena/end/x", gaussian->arenaRect.endPoint.px);
		n.getParam("plumesim/arena/end/y", gaussian->arenaRect.endPoint.py);
		n.getParam("plumesim/cell_size", gaussian->cellSize);
		n.getParam("plumesim/diffx", gaussian->diffX);
		n.getParam("plumesim/diffy", gaussian->diffY);
		n.getParam("plumesim/radius", gaussian->radius);
		n.getParam("plumesim/max_points_per_cell", gaussian->maxPointsPerCell);
	}
	else if(type.compare("fluent") == 0)
	{
		source = new PSFluent();
		PSFluent * fluent = dynamic_cast<PSFluent*>(source);
		
		n.getParam("plumesim/max_points_per_cell", fluent->maxPointsPerCell);
		n.getParam("plumesim/file_name", fluent->fileName);
		n.getParam("plumesim/num_of_frames", fluent->numOfFrames);
		n.getParam("plumesim/cell_size", fluent->cellSize);
		n.getParam("plumesim/max_concentration", fluent->maxOdorValue);
	}
	else if(type.compare("pslog") == 0)
	{
		source = new PSLog();
		PSLog * log = dynamic_cast<PSLog*>(source);
		
		n.getParam("plumesim/max_points_per_cell", log->maxPointsPerCell);
		n.getParam("plumesim/file_name", log->logFilePath);
	}
	else
	{
		ROS_FATAL("The type of simulation selected is not defined!");
		ROS_BREAK();
	}
	
	// Setting the color for the plume
	n.param("plumesim/color/r", this->plume_color_r, 1.0);
	n.param("plumesim/color/g", this->plume_color_g, 0.0);
	n.param("plumesim/color/b", this->plume_color_b, 0.0);
	n.param("plumesim/color/a", this->plume_color_a, 0.9);
	
	// Setting the plume identifiers
	plume_ns = "plume";
	n.getParam("plumesim/ns", plume_ns);
	n.param("plumesim/id", plume_id, 0);
	
	// Setting the plume frequency
	n.param("plumesim/frequency", freq, 1.0);
	
	if(source->setup() == -1)
	{
		ROS_FATAL("Failed to setup the plume!");
		ROS_BREAK();
	}
	ROS_INFO("Plume setup is complete.");

	// Set the markers fields...
	points.header.frame_id = "/plume_frame";
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;
	points.ns = this->plume_ns;
	points.id = this->plume_id;
	points.pose.orientation.w = 1.0;
	points.type = visualization_msgs::Marker::SPHERE_LIST;
	// POINTS markers use x and y scale for width/height respectively
	// SPHERE_LIST also uses z
	points.scale.x = 0.01;
	points.scale.y = 0.01;
	points.scale.z = 0.01;
	// Points color
	points.color.r = this->plume_color_r;
	points.color.g = this->plume_color_g;
	points.color.b = this->plume_color_b;
	points.color.a = this->plume_color_a;
}

// *****************************************************************************
// Destructor.
PlumeSim::~PlumeSim()
{
	source->cleanup();
	delete source;
}

// *****************************************************************************
// Main function for the PlumeSim class
void PlumeSim::UpdatePlume()
{	
	if(this->source->IsPlaying())
	{
		if(this->source->generatePoints() == -1)
		{
			ROS_ERROR("Unable to generate a new plume!");
		}
		else
		{
			points.header.stamp = ros::Time::now();
			points.points.clear();
			
			for(int i=0 ; i<source->plumePoints.size() ; i++)
			{
				geometry_msgs::Point p;
				p.x = source->plumePoints[i].px;
				p.y = source->plumePoints[i].py;
				p.z = source->plumePoints[i].pz;
				points.points.push_back(p);
			}
		}
	}
	// Publish data
	marker_pub.publish(points);
}

// *****************************************************************************
// Get the frequency of the plume.
double PlumeSim::getFrequency()
{
	return this->freq;
}

// *****************************************************************************
// ReadPlumeSim service.
bool PlumeSim::ReadPlumeSim(plumesim::ReadPlumeSim::Request  &req, plumesim::ReadPlumeSim::Response &res)
{
	PSOdorData odor_data;
	PSPoint3d point;
	
	point.px = req.odom.pose.pose.position.x;
	point.py = req.odom.pose.pose.position.y;
	point.pz = req.odom.pose.pose.position.z;
	
	source->getChemicalReading(&point, &odor_data);
	
	res.sniff.header.frame_id = "/plume_frame";
	res.sniff.header.stamp = ros::Time::now();
	res.sniff.sensor_model.resize(1);
	res.sniff.sensor_model[0] = "plumesim_sensor";
	res.sniff.reading.resize(1);
	res.sniff.reading[0] = odor_data.chemical;
	res.sniff.temperature_c = 21;
	res.sniff.temperature_f = 69.8;
	res.sniff.relative_humidity = 30;

	// TODO: Put odor_data.windSpeed and odor_data.windDirection into the proper message...

	return true;
}


// *****************************************************************************
// Main function for the PlumeSim node
int main(int argc, char** argv)
{
	ros::init(argc, argv, "plumesim");

	PlumeSim * plumesim_node = new PlumeSim();
	
	boost::thread t = boost::thread::thread(boost::bind(&ros::spin));

  	ros::Rate r(plumesim_node->getFrequency());
  	while(ros::ok())
  	{
      		plumesim_node->UpdatePlume();
      		r.sleep();
    	}
  	t.join();

	delete plumesim_node;

  	exit(0);
}

// EOF

