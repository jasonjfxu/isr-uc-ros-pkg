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
* Author: Gonçalo Cabrita on 05/03/2012
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

ros::Publisher * marker_pub_;

void poleStructureCallback(const pole_structure_mapper::PoleStructure::ConstPtr& msg)
{
    //ROS_INFO("Got a structure msg!!!");

    pole_structure_mapper::PoleStructure pole_structure_ = *msg;
    
    for(int i=0 ; i<pole_structure_.pole.size() ; i++)
    {      
        char tag[16];
        sprintf(tag, "pole_%d", i);
        
        // Display the cylinder
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
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
        marker_pub_->publish(marker);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pole_structure_visualizer_node");
    
    ROS_INFO("Pole Structure Visualizer for ROS v0.1");
    
    // ROS stuff
	ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("pole_structure", 1, poleStructureCallback);
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("pole_structure_visualizer", 30);
    marker_pub_ = &pub;

    ros::spin();

}

// EOF

