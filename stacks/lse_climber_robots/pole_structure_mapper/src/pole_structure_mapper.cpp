/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 03/10/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#include <pole_structure_mapper/PoleSectionStamped.h>
#include <pole_structure_mapper/PoleStructure.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

class PoleMapper
{
	public:
	PoleMapper();
	~PoleMapper();
	
	void displayStructure();
	void publishStructure();

	private:
	void poleSectionCallback(const pole_structure_mapper::PoleSectionStamped::ConstPtr& msg);
    
    	bool polesAreTheSame(pole_structure_mapper::PoleSection * pole1, pole_structure_mapper::PoleSection * pole2);
	void getPointInAxis(pole_structure_mapper::PoleSection * pole, geometry_msgs::Point * p, geometry_msgs::Point * p_in_axis);
    
	// ROS stuff
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
    
    	// Global frame_id
    	std::string global_frame_id_;
    
    	// Thresholds
    	double diameter_threshold_;
    	double axis_threshold_;
    	double distance_threshold_;
    	double length_threshold_;
    
	//ros::Subscriber cloud_sub_;
    	message_filters::Subscriber<pole_structure_mapper::PoleSectionStamped> pole_sub_;
    	tf::TransformListener tf_;
    	tf::MessageFilter<pole_structure_mapper::PoleSectionStamped> * tf_filter_;
    
	ros::Publisher pole_structure_pub_;
    	ros::Publisher marker_pub_;
    
    	pole_structure_mapper::PoleStructure pole_structure_;
};

PoleMapper::PoleMapper() : nh_(), pnh_("~")
{	
    pnh_.param<std::string>("global_frame_id", global_frame_id_, "/map");
    pnh_.param("diameter_threshold", diameter_threshold_, 0.03);
    pnh_.param("axis_threshold", axis_threshold_, 10.0);
    axis_threshold_ = axis_threshold_*M_PI/180.0;
    pnh_.param("distance_threshold", distance_threshold_, 0.10);
    pnh_.param("length_threshold", length_threshold_, 0.60);
    
    pole_sub_.subscribe(nh_, "/pole", 1);
	tf_filter_ = new tf::MessageFilter<pole_structure_mapper::PoleSectionStamped>(pole_sub_, tf_, global_frame_id_, 10);
    tf_filter_->registerCallback( boost::bind(&PoleMapper::poleSectionCallback, this, _1) );
    
	pole_structure_pub_ = nh_.advertise<pole_structure_mapper::PoleStructure>("pole_structure", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pole_structure_visualizer", 1);
    
    pole_structure_.header.frame_id = global_frame_id_;
}

PoleMapper::~PoleMapper()
{
	
}

void PoleMapper::poleSectionCallback(const pole_structure_mapper::PoleSectionStamped::ConstPtr& msg)
{
    ROS_INFO("Pole Structure Mapper - %s - Got a pole section...", __FUNCTION__);
    
	if(msg->pole.length < length_threshold_) return;

	// 1. Transform pole section into global frame
    geometry_msgs::PointStamped cylinder_point, map_point;
    
    cylinder_point.header = msg->header;
    cylinder_point.point = msg->pole.base;
    
    try { tf_.transformPoint(global_frame_id_, cylinder_point, map_point); }
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("Pole Structure Mapper- %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
    
    geometry_msgs::Vector3Stamped cylinder_axis, map_axis;
    
    cylinder_axis.header = msg->header;
    cylinder_axis.vector = msg->pole.axis;
    
    try { tf_.transformVector(global_frame_id_, cylinder_axis, map_axis); }
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("Pole Structure Mapper- %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
    
    pole_structure_mapper::PoleSection map_pole_section;
    map_pole_section.base = map_point.point;
    map_pole_section.axis = map_axis.vector;
    map_pole_section.length = msg->pole.length;
    map_pole_section.diameter = msg->pole.diameter;
    
    // 2. Check if pole exists in the current structure
    bool found_pole = false;
    for(int i=0 ; i<pole_structure_.pole.size() ; i++)
    {
        // 2.1 If it already exists update it
        if(polesAreTheSame(&pole_structure_.pole[i], &map_pole_section))
        {
            ROS_INFO("Poles are the same!!!");

            // Update pole position and length!
            geometry_msgs::Point points[4];
            
            points[0].x = pole_structure_.pole[i].base.x;
            points[0].y = pole_structure_.pole[i].base.y;
            points[0].z = pole_structure_.pole[i].base.z;
            
            points[1].x = pole_structure_.pole[i].base.x + pole_structure_.pole[i].length * pole_structure_.pole[i].axis.x;
            points[1].y = pole_structure_.pole[i].base.y + pole_structure_.pole[i].length * pole_structure_.pole[i].axis.y;
            points[1].z = pole_structure_.pole[i].base.z + pole_structure_.pole[i].length * pole_structure_.pole[i].axis.z;
            
            geometry_msgs::Point top2, base2;
            points[3].x = map_pole_section.base.x + map_pole_section.length * map_pole_section.axis.x;
            points[3].y = map_pole_section.base.y + map_pole_section.length * map_pole_section.axis.y;
            points[3].z = map_pole_section.base.z + map_pole_section.length * map_pole_section.axis.z;
            getPointInAxis(&pole_structure_.pole[i], &points[3], &points[3]);
            getPointInAxis(&pole_structure_.pole[i], &map_pole_section.base, &points[2]);
            
            geometry_msgs::Point cylinder_top;
            
            double lowest_t = 0.0;
            double highest_t = 0.0;
            
            for(int j=1 ; j<4 ; j++)
            {
                double t = (points[j].x - pole_structure_.pole[i].base.x)/pole_structure_.pole[i].axis.x;
                
                if(t<lowest_t)
                {
                    pole_structure_.pole[i].base.x = points[j].x;
                    pole_structure_.pole[i].base.y = points[j].y;
                    pole_structure_.pole[i].base.z = points[j].z;
                    lowest_t = t;
                }
                
                if(t>highest_t)
                {
                    cylinder_top.x = points[j].x;
                    cylinder_top.y = points[j].y;
                    cylinder_top.z = points[j].z;
                    highest_t = t;
                }
            }
            
            // New length
            pole_structure_.pole[i].length = sqrt(pow(cylinder_top.x - pole_structure_.pole[i].base.x, 2) + pow(cylinder_top.y - pole_structure_.pole[i].base.y, 2) + pow(cylinder_top.z - pole_structure_.pole[i].base.z, 2));
            
            // Average diameter
            pole_structure_.pole[i].diameter = (pole_structure_.pole[i].diameter + map_pole_section.diameter) / 2.0;
            
            // Linear interpolation of both axis
	    tf::Vector3 pi(pole_structure_.pole[i].axis.x, pole_structure_.pole[i].axis.y, pole_structure_.pole[i].axis.z);
            tf::Vector3 p(map_pole_section.axis.x, map_pole_section.axis.y, map_pole_section.axis.z);
            pi = pi.lerp(p, 0);
	    pole_structure_.pole[i].axis.x = pi.x();
	    pole_structure_.pole[i].axis.y = pi.y();
	    pole_structure_.pole[i].axis.z = pi.z();
            
            // Average base point
            // TODO: Finish this...
            
            found_pole = true;
            break;
        }
    }
    
    // 2.2 If not add it
    if(!found_pole)
    {
        ROS_INFO("Poles are not the same!!!");

        pole_structure_.pole.push_back(map_pole_section);
    }
    
    pole_structure_.header.stamp = ros::Time::now();
}

bool PoleMapper::polesAreTheSame(pole_structure_mapper::PoleSection * pole1, pole_structure_mapper::PoleSection * pole2)
{
    // 1. Check the diameter
    if(fabs(pole1->diameter - pole2->diameter) > diameter_threshold_)
       return false;

	ROS_INFO("Passed diameter...");
    
    // 2. Check the axis
    tf::Vector3 p1(pole1->axis.x, pole1->axis.y, pole1->axis.z);
    tf::Vector3 p2(pole2->axis.x, pole2->axis.y, pole2->axis.z);
    double alpha = fabs(acos(p1.dot(p2)));
    if(axis_threshold_ < alpha && alpha < M_PI-axis_threshold_)
        return false;

	ROS_INFO("Passed axis...");
    
    // 3. Check the position
    geometry_msgs::Point p;
    getPointInAxis(pole1, &pole2->base, &p);
    
    double distance = sqrt(pow(pole2->base.x - p.x, 2) + pow(pole2->base.y - p.y, 2) + pow(pole2->base.z - p.z,2));
    
    if(distance > distance_threshold_) return false;

	ROS_INFO("Passed position...");
       
    return true;
}

void PoleMapper::getPointInAxis(pole_structure_mapper::PoleSection * pole, geometry_msgs::Point * p, geometry_msgs::Point * p_in_axis)
{
    double t = (pole->axis.x*(p->x-pole->base.x) + pole->axis.y*(p->y-pole->base.y) + pole->axis.z*(p->z-pole->base.z))/(pow(pole->axis.x,2) + pow(pole->axis.y,2) + pow(pole->axis.z,2));
    
    p_in_axis->x = pole->base.x + pole->axis.x * t;
    p_in_axis->y = pole->base.y + pole->axis.y * t;
    p_in_axis->z = pole->base.z + pole->axis.z * t;
}

void PoleMapper::publishStructure()
{
	pole_structure_pub_.publish(pole_structure_);
}

void PoleMapper::displayStructure()
{
    for(int i=0 ; i<pole_structure_.pole.size() ; i++)
    {
        char tag[16];
        sprintf(tag, "pole_%d", i);
        
        // Display the cylinder
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame_id_;
        marker.header.stamp = ros::Time::now();
        
        marker.ns = tag;
        marker.id = 0;
        
        marker.type = visualization_msgs::Marker::CYLINDER;
        
        marker.action = visualization_msgs::Marker::ADD;
        
        // For the marker the position is the middle point between the bottom and the top
        marker.pose.position.x = pole_structure_.pole[i].base.x + pole_structure_.pole[i].length/2.0 * pole_structure_.pole[i].axis.x;
        marker.pose.position.y = pole_structure_.pole[i].base.y + pole_structure_.pole[i].length/2.0 * pole_structure_.pole[i].axis.y;
        marker.pose.position.z = pole_structure_.pole[i].base.z + pole_structure_.pole[i].length/2.0 * pole_structure_.pole[i].axis.z;

	tf::Vector3 axis_vector(pole_structure_.pole[i].axis.x, pole_structure_.pole[i].axis.y, pole_structure_.pole[i].axis.z);
	tf::Vector3 up_vector(0.0, 0.0, 1.0);
    	tf::Vector3 right_vector = axis_vector.cross(up_vector);
    	right_vector.normalized();
    	tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    	q.normalize();
    	geometry_msgs::Quaternion cylinder_orientation;
    	tf::quaternionTFToMsg(q, cylinder_orientation);

        marker.pose.orientation = cylinder_orientation;
        
        marker.scale.x = pole_structure_.pole[i].diameter;
        marker.scale.y = pole_structure_.pole[i].diameter;
        marker.scale.z = pole_structure_.pole[i].length;
        
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8;
        
        marker.lifetime = ros::Duration();
        
        // Publish the marker
        marker_pub_.publish(marker);
    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pole_structure_mapper_node");
    
	ROS_INFO("Pole Structure Mapper for ROS v0.1");
	
	PoleMapper pm;
	
	ros::Rate r(1.0);
	while(ros::ok())
	{
		pm.publishStructure();
		pm.displayStructure();
		
		ros::spinOnce();
		r.sleep();
	}
}

// EOF

