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
* Author: Gonçalo Cabrita on 17/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>

//#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <rtk_ros/UTMConversion.h>

double true_north_compensation;

tf::TransformListener * tf_listener;

std::string global_frame_id;
std::string odom_frame_id;

tf::Transform t_world_imu;
tf::Transform t_world_gps;

tf::Transform t_odom_imu;
tf::Transform t_odom_gps;

void imuCallback(const boost::shared_ptr<const sensor_msgs::Imu>& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::PoseStamped imu_pose;
            
    try 
    {
        tf_listener->transformPose(odom_frame_id, pose, imu_pose);
    }
    catch(tf::TransformException &ex) 
    {
        ROS_ERROR("Squirtle Localization - %s - Error: %s", __FUNCTION__, ex.what());
        return;
    }
    
    tf::Quaternion q_imu;
    tf::quaternionMsgToTF(imu_pose.pose.orientation, q_imu);
    t_world_imu.setRotation(q_imu + tf::createQuaternionFromYaw(M_PI/2.0 + true_north_compensation));

    t_odom_imu.setOrigin(tf::Vector3(imu_pose.pose.position.x, imu_pose.pose.position.y, imu_pose.pose.position.z));
    t_odom_imu.setRotation(q_imu);
}

void gpsCallback(const boost::shared_ptr<const sensor_msgs::NavSatFix>& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    geometry_msgs::PoseStamped gps_pose;
            
    try 
    {
        tf_listener->transformPose(odom_frame_id, pose, gps_pose);
    }
    catch(tf::TransformException &ex) 
    {
        ROS_ERROR("Squirtle Localization - %s - Error: %s", __FUNCTION__, ex.what());
        return;
    }

    double x, y;
    int zone_number; char zone_letter;
    GPStoUTM(msg->latitude, msg->longitude, y, x, zone_number, zone_letter);
    t_world_gps.setOrigin(tf::Vector3(x, y, msg->altitude));

    t_odom_gps.setOrigin(tf::Vector3(gps_pose.pose.position.x, gps_pose.pose.position.y, gps_pose.pose.position.z));
    tf::Quaternion q_gps;
    tf::quaternionMsgToTF(gps_pose.pose.orientation, q_gps);
    t_odom_gps.setRotation(q_gps);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "squirtle_localization_node");

	ROS_INFO("Squirtle Localization for ROS v0.1");

    ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double rate;
	pn.param("rate", rate, 2.0);

	pn.param("true_north__compensation", true_north_compensation, 0.0499164166);
    
	pn.param<std::string>("global_frame_id", global_frame_id, "/world");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	
	tf_listener = new tf::TransformListener();

	message_filters::Subscriber<sensor_msgs::Imu> * imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>();
	imu_sub->subscribe(n, "imu", 20);
    tf::MessageFilter<sensor_msgs::Imu> * tf_filter_imu = new tf::MessageFilter<sensor_msgs::Imu>(*imu_sub, *tf_listener, odom_frame_id, 20);
    tf_filter_imu->registerCallback( boost::bind(imuCallback, _1) );

	message_filters::Subscriber<sensor_msgs::NavSatFix> * gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix>();
	gps_sub->subscribe(n, "gps/fix", 20);
    tf::MessageFilter<sensor_msgs::NavSatFix> * tf_filter_gps = new tf::MessageFilter<sensor_msgs::NavSatFix>(*gps_sub, *tf_listener, odom_frame_id, 20);
    tf_filter_gps->registerCallback( boost::bind(gpsCallback, _1) );

	/*message_filters::Subscriber<nav_msgs::Odometry> * odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>();
	odom_sub->subscribe(n, "odom", 20);
    tf::MessageFilter<nav_msgs::Odometry> * tf_filter = new tf::MessageFilter<nav_msgs::Odometry>(*odom_sub, *tf_listener, base_footprint_frame_id, 20);
    tf_filter->registerCallback( boost::bind(odomCallback, _1) );*/

	static tf::TransformBroadcaster tf_broadcaster;

	ros::Rate r(rate);
	while(n.ok())
	{
        tf::Transform t_world_odom_rotation = t_world_imu * t_odom_imu.inverse();
        tf::Transform t_world_odom_translation = t_world_gps * t_odom_gps.inverse();

        tf::Transform t_world_odom;
        t_world_odom.setOrigin(t_world_odom_translation.getOrigin());
        t_world_odom.setRotation(t_world_odom_rotation.getRotation());

        tf_broadcaster.sendTransform(tf::StampedTransform(t_world_odom, ros::Time::now(), global_frame_id, odom_frame_id));

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

// EOF
