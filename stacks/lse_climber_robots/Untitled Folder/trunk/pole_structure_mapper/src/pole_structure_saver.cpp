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

#include <stdio.h>

bool done;

void poleStructureCallback(const pole_structure_mapper::PoleStructure::ConstPtr& msg)
{
    if(done) return;

    pole_structure_mapper::PoleStructure pole_structure_ = *msg;
    
    ROS_INFO("Got a pole structure message, writting to file...");
    
    FILE * pFile;
        
    pFile = fopen ("pole_structure.txt", "w");
        
    if(pFile==NULL)
    {
    	ROS_ERROR("Failed to open the file!");
        fclose(pFile);
        return;
    }

    for(int i=0 ; i<pole_structure_.pole.size() ; i++)
    {
        
        fprintf(pFile, "Base x:%lf y:%lf z%lf - Axis x:%lf y:%lf z%lf - Length: %lf - Diameter: %lf\n", pole_structure_.pole[i].base.x, pole_structure_.pole[i].base.y, pole_structure_.pole[i].base.z, pole_structure_.pole[i].axis.x, pole_structure_.pole[i].axis.y, pole_structure_.pole[i].axis.z, pole_structure_.pole[i].length, pole_structure_.pole[i].diameter);
    }
    
    fclose(pFile);

    done = true;
    ROS_INFO("Done! Press ctrl+c to exit!");
}

int main(int argc, char** argv)
{
	done = false;

	ros::init(argc, argv, "pole_structure_saver_node");
    
    ROS_INFO("Pole Structure Saver for ROS v0.1");
    
    // ROS stuff
	ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("pole_structure", 1, poleStructureCallback);
        
    ros::spin();

}

// EOF

