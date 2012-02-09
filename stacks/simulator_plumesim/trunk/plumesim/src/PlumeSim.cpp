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
* Author: Gonçalo Cabrita and Pedro Sousa on 31/05/2010
*********************************************************************/
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

using namespace plumesim;

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
	
	std::string global_frame_id;
	
	double freq;
	
	double plume_color_r, plume_color_g, plume_color_b, plume_color_a;
	
	std::string plume_ns;
	int plume_id;
};

// *****************************************************************************
// Constructor.
PlumeSim::PlumeSim() : n("~")
{		
	ROS_INFO("PlumeSim for ROS %.2f", NODE_VERSION);
	
	marker_pub = n.advertise<visualization_msgs::Marker>("plumesim_markers", 10);
	service = n.advertiseService("read_plumesim", &PlumeSim::ReadPlumeSim, this);
	
	// Setting the plume type
	if(!n.hasParam("type"))
	{
		ROS_FATAL("No type of simulation was defined!");
		ROS_BREAK();
	}
	std::string type;
	n.getParam("type", type);
	
	if(type.compare("meadering") == 0)
	{
		source = new PSMeadering();
		PSMeadering * meandering = dynamic_cast<PSMeadering*>(source);		
		
		n.param("releasepoint/x", meandering->startPoint.px, 0.0);
		n.param("releasepoint/y", meandering->startPoint.py, 0.0);
		n.param("releasepoint/z", meandering->startPoint.pz, 0.0);
		n.param("arena/start/x", meandering->arenaRect.startPoint.px, 0.0);
		n.param("arena/start/y", meandering->arenaRect.startPoint.py, 0.0);
		n.param("arena/end/x", meandering->arenaRect.endPoint.px, 0.0);
		n.param("arena/end/y", meandering->arenaRect.endPoint.py, 0.0);
		n.param("number_of_points", meandering->numOfPlumePoints, 100);
		n.param("release_rate", meandering->releaseRate, 10.0);
		n.param("dispersion_coeficient", meandering->dispersionCoeficient, 0.06);
		n.param("radius", meandering->radius, 0.1);
	}
	else if(type.compare("gaussian") == 0)
	{
		source = new PSGaussian();
		PSGaussian * gaussian = dynamic_cast<PSGaussian*>(source);
		
		n.getParam("releasepoint/x", gaussian->startPoint.px);
		n.getParam("releasepoint/y", gaussian->startPoint.py);
		n.getParam("releasepoint/z", gaussian->startPoint.pz);
		n.getParam("arena/start/x", gaussian->arenaRect.startPoint.px);
		n.getParam("arena/start/y", gaussian->arenaRect.startPoint.py);
		n.getParam("arena/end/x", gaussian->arenaRect.endPoint.px);
		n.getParam("arena/end/y", gaussian->arenaRect.endPoint.py);
		n.getParam("cell_size", gaussian->cellSize);
		n.getParam("diffx", gaussian->diffX);
		n.getParam("diffy", gaussian->diffY);
		n.getParam("radius", gaussian->radius);
		n.getParam("max_points_per_cell", gaussian->maxPointsPerCell);
	}
	else if(type.compare("fluent") == 0)
	{
		source = new PSFluent();
		PSFluent * fluent = dynamic_cast<PSFluent*>(source);
		
		n.getParam("max_points_per_cell", fluent->maxPointsPerCell);
		n.getParam("file_name", fluent->fileName);
		n.getParam("num_of_frames", fluent->numOfFrames);
		n.getParam("cell_size", fluent->cellSize);
		n.getParam("max_concentration", fluent->maxOdorValue);
	}
	else if(type.compare("pslog") == 0)
	{
		source = new PSLog();
		PSLog * log = dynamic_cast<PSLog*>(source);
		
		n.getParam("max_points_per_cell", log->maxPointsPerCell);
		n.getParam("file_name", log->logFilePath);
	}
	else
	{
		ROS_FATAL("The type of simulation selected is not defined!");
		ROS_BREAK();
	}
	
	n.param<std::string>("global_frame_id", global_frame_id, "map");
	
	// Setting the color for the plume
	n.param("color/r", this->plume_color_r, 1.0);
	n.param("color/g", this->plume_color_g, 0.0);
	n.param("color/b", this->plume_color_b, 0.0);
	n.param("color/a", this->plume_color_a, 0.9);
	
	// Setting the plume identifiers
	plume_ns = "plume";
	n.getParam("ns", plume_ns);
	n.param("id", plume_id, 0);
	
	// Setting the plume frequency
	n.param("frequency", freq, 1.0);
	
	if(source->setup() == -1)
	{
		ROS_FATAL("Failed to setup the plume!");
		ROS_BREAK();
	}
	ROS_INFO("Plume setup is complete.");

	// Set the markers fields...
	points.header.frame_id = global_frame_id;
	points.header.stamp = ros::Time::now();
	points.action = visualization_msgs::Marker::ADD;
	points.ns = this->plume_ns;
	points.id = this->plume_id;
	points.pose.orientation.w = 1.0;
	points.type = visualization_msgs::Marker::SPHERE_LIST;
	// POINTS markers use x and y scale for width/height respectively
	// SPHERE_LIST also uses z
	points.scale.x = 0.06;
	points.scale.y = 0.06;
	points.scale.z = 0.06;
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
				//p.z = source->plumePoints[i].pz;
				p.z = 0.2;
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
	
	res.nostril.header.frame_id = req.odom.header.frame_id;
	res.nostril.header.stamp = ros::Time::now();
	res.nostril.sensor_model = "PlumeSim";
	res.nostril.reading = odor_data.chemical;
	res.nostril.max_reading = 1.0;
	res.nostril.min_reading = 0.0;

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

