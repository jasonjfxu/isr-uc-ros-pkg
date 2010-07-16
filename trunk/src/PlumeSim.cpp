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


#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "plumesim/ReadPlumeSim.h"

// Types of sources
#include "utils/PSSources.h"


bool ReadPlumeSim(plumesim::ReadPlumeSim::Request  &req, plumesim::ReadPlumeSim::Response &res)
{
	
	
	return true;
}


// *****************************************************************************
// Main function for the PlumeSim node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "plumesim");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::ServiceServer service = n.advertiseService("read_plumesim", ReadPlumeSim);
	
	int freq;
	n.param("plumesim/frequency", freq, 1);

	ros::Rate r(freq);
	
	ROS_INFO("PlumeSim for ROS");
	
	PSSource * source;

	// Setting the plume type
	if(!n.hasParam("plumesim/type"))
	{
		ROS_FATAL("No type of simulation was defined!");
    		ROS_BREAK();
	}

	std::string type;
	n.getParam("plumesim/type", type);

	ROS_INFO("Type of plume: %s", type.c_str());
	
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
	double color_r, color_g, color_b, color_a;
	n.param("plumesim/color/r", color_r, 1.0);
	n.param("plumesim/color/g", color_g, 0.0);
	n.param("plumesim/color/b", color_b, 0.0);
	n.param("plumesim/color/a", color_a, 0.9);
		
	if( source->setup() == -1)
	{
		ROS_FATAL("Failed to setup the plume!");
    		ROS_BREAK();
	}

	// The main loop
	// Interact with the device here
	while(ros::ok())
	{
		visualization_msgs::Marker points;
		points.header.frame_id = "/plume_frame";
		points.header.stamp = ros::Time::now();
		points.action = visualization_msgs::Marker::ADD;
		points.ns = "plume";
		points.id = 0;
		points.pose.orientation.w = 1.0;
		points.type = visualization_msgs::Marker::POINTS;
		//SPHERE_LIST;
		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.01;
		points.scale.y = 0.01;
		//points.scale.z = 0.01;
		// Points color
		points.color.r = color_r;
		points.color.g = color_g;
		points.color.b = color_b;
		points.color.a = color_a;
		
		if(source->IsPlaying())
		{
			if( source->generatePoints() == -1)
			{
				ROS_ERROR("Unable to generate a new plume!");
			}
		}
		
		for(int i=0 ; i<source->plumePoints.size() ; i++)
		{
			geometry_msgs::Point p;
			p.x = source->plumePoints[i].px;
			p.y = source->plumePoints[i].py;
			p.z = source->plumePoints[i].pz;
			points.points.push_back(p);
		}
		// Publish data
		marker_pub.publish(points);

		r.sleep();
	}

	source->cleanup();
	delete source;
}

// EOF

