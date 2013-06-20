/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 06/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <rtk_msgs/UTMCoordinates.h>

rtk_msgs::UTMCoordinates utm_base_station;

void utmCallback(const rtk_msgs::UTMCoordinates::ConstPtr& msg)
{
    utm_base_station = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_broadcaster");

	ROS_INFO("BaseStation tf broadcaster for ROS v0.1");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	std::string global_frame_id;
	std::string base_station_frame_id;

	pn.param<std::string>("global_frame_id", global_frame_id, "/world");
	pn.param<std::string>("base_station_frame_id", base_station_frame_id, "base_station");

	ros::Subscriber utm_sub;
    	if(pn.getParam("base_position/x", utm_base_station.position.x) && pn.getParam("base_position/y", utm_base_station.position.y) && pn.getParam("base_position/z", utm_base_station.position.z))
	{
		ROS_INFO("BaseStation tf broadcaster -- Loading base station parameters from the parameter server.");

		XmlRpc::XmlRpcValue position_covariance;
		if( pn.getParam("base_position/covariance", position_covariance) )
		{
		    ROS_ASSERT(position_covariance.getType() == XmlRpc::XmlRpcValue::TypeArray);

		    if(position_covariance.size() != 9)
		    {
			ROS_WARN("RTK -- The base station covariances are not complete! Using default values...");
		    }
		    else
		    {
			for(int i=0 ; i<position_covariance.size() ; ++i)
			{
			    ROS_ASSERT(position_covariance[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

			    utm_base_station.position_covariance[i] = static_cast<double>(position_covariance[i]);
			}
		    }
		}
	}
	else
	{
		ROS_INFO("BaseStation tf broadcaster -- Subscribing to the base station for online parameters.");

		utm_sub = n.subscribe("base_station/gps/utm", 50, utmCallback);
	}

	ros::Rate r(50.0);
	while(n.ok())
	{
		static tf::TransformBroadcaster tf_broadcaster;
		
		tf::Transform t;
		t.setOrigin(tf::Vector3(utm_base_station.position.x, utm_base_station.position.y, utm_base_station.position.z));
		t.setRotation(tf::createQuaternionFromYaw(0.0));
		tf_broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), global_frame_id, base_station_frame_id));	

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
