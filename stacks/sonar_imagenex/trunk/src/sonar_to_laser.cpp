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
* Author: Gonçalo Cabrita on 12/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#define EPSILON 0.0174532925

tf::TransformListener * tf_listener;

ros::Publisher * laser_pub_ptr;

sensor_msgs::LaserScan scan_msg;

std::string sonar_frame_id;

double angle_upper_bound;
double angle_lower_bound;

ros::Time first_scan;

bool areEqual(double a, double b)
{
    return fabs(a - b) < EPSILON;
}

void startLaserMsg(double range, double min_range, double max_range, double angle)
{
	scan_msg.ranges.empty();
	scan_msg.ranges.push_back(range);
	scan_msg.range_min = min_range;
	scan_msg.range_max = max_range;
	scan_msg.angle_min = angle;

	first_scan = ros::Time::now();
}

void publishLaserMsg(double range, double angle)
{
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.ranges.push_back(range);
	scan_msg.angle_max = angle;
	ros::Duration total_time = ros::Time::now() - first_scan;
	scan_msg.time_increment = total_time.toSec() / scan_msg.ranges.size();
	scan_msg.scan_time = total_time.toSec();

	laser_pub_ptr->publish(scan_msg);
}

void rangeCallback(const boost::shared_ptr<const sensor_msgs::Range>& msg)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = msg->header.frame_id;
	pose.header.stamp = msg->header.stamp;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	geometry_msgs::PoseStamped sonar_pose;
	    
	try 
	{
		tf_listener->transformPose(sonar_frame_id, pose, sonar_pose);
	}
	catch(tf::TransformException &ex) 
	{
		ROS_ERROR("Range2Laser - %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}

	if(areEqual(tf::getYaw(sonar_pose.pose.orientation), angle_lower_bound))
	{
		if(scan_msg.ranges.size() > 0)
		{
			publishLaserMsg(msg->range, tf::getYaw(sonar_pose.pose.orientation));
		}

		startLaserMsg(msg->range, msg->min_range, msg->max_range, tf::getYaw(sonar_pose.pose.orientation));
	}
	else if(areEqual(tf::getYaw(sonar_pose.pose.orientation), angle_upper_bound) && scan_msg.ranges.size() > 0)
	{
		publishLaserMsg(msg->range, tf::getYaw(sonar_pose.pose.orientation));

		startLaserMsg(msg->range, msg->min_range, msg->max_range, tf::getYaw(sonar_pose.pose.orientation));
	}
	else if(scan_msg.ranges.size() > 0)
	{
		scan_msg.ranges.push_back(msg->range);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "range_to_laser");

	ROS_INFO("Range2Laser for ROS v0.1");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double angle_upper_bound;
	double angle_lower_bound;
	pn.param("angle_upper_bound", angle_upper_bound, 2.35619449);
	pn.param("angle_lower_bound", angle_lower_bound, 0.785398163);
	
	pn.param<std::string>("sonar_frame_id", sonar_frame_id, "sonar");
	
	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laser_scan", 10);
	laser_pub_ptr = &laser_pub;

	tf_listener = new tf::TransformListener();

	message_filters::Subscriber<sensor_msgs::Range> * range_sub = new message_filters::Subscriber<sensor_msgs::Range>();
	range_sub->subscribe(n, "sonar_range", 100);
        tf::MessageFilter<sensor_msgs::Range> * tf_filter = new tf::MessageFilter<sensor_msgs::Range>(*range_sub, *tf_listener, sonar_frame_id, 20);
        tf_filter->registerCallback( boost::bind(rangeCallback, _1) );

    	scan_msg.header.frame_id = sonar_frame_id;
    
	ros::spin();
	
	return 0;
}

// EOF
