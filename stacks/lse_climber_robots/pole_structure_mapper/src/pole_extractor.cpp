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
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/Marker.h>
#include <pole_structure_mapper/PoleSectionStamped.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#define SLICE_SAMPLES 3

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PoleExtractor
{
	public:
	PoleExtractor();
	~PoleExtractor();
	
	bool cylinderSegmentation();
	
	private:
	void pointCloudCallback(const PointCloud::ConstPtr& msg);
	void controlCallback(const std_msgs::Bool::ConstPtr& msg);
    
    void getMinMax3DalongAxis(const PointCloud::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT * axis_point, tf::Vector3 * normal);
	
	// ROS stuff
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
    
    	// Global frame_id
    	std::string gripper_tag_frame_id_;

	// Size of the crop box
	double filter_box_size_;

	double min_diameter_;
	double max_diameter_;

	double normal_distance_weight_;
	int max_iterations_;
	double distance_threshold_;
    
	//ros::Subscriber cloud_sub_;
    	message_filters::Subscriber<PointCloud> cloud_sub_;
    	tf::TransformListener tf_;
    	tf::MessageFilter<PointCloud> * tf_filter_;
    
	ros::Publisher cylinder_pub_;
	ros::Publisher roi_cloud_pub_;
    	ros::Publisher marker_pub_;
    	ros::Publisher pole_pub_;

	ros::Subscriber control_sub_;
	
	// All the objects needed
	pcl::NormalEstimation<PointT, pcl::Normal> ne_;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_; 
	pcl::ExtractIndices<PointT> extract_;
	pcl::ExtractIndices<pcl::Normal> extract_normals_;
	pcl::KdTreeFLANN<PointT>::Ptr tree_;
	
	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud_filtered_;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
	pcl::ModelCoefficients::Ptr coefficients_cylinder_;
	pcl::PointIndices::Ptr inliers_cylinder_;
	
	bool got_cloud_;

	double max(double a, double b);
	double min(double a, double b);

	bool run_;
};

PoleExtractor::PoleExtractor() : nh_(), pnh_("~")
{
	got_cloud_ = false;

	// Alloc space...
	tree_ = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
	cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	coefficients_cylinder_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	inliers_cylinder_ = pcl::PointIndices::Ptr(new pcl::PointIndices);
	
    	pnh_.param<std::string>("gripper_tag_frame_id", gripper_tag_frame_id_, "ar_marker");
	pnh_.param("filter_box_size", filter_box_size_, 2.0);
	pnh_.param("min_diameter", min_diameter_, 0.10);
	pnh_.param("max_diameter", max_diameter_, 0.20);
	pnh_.param("normal_distance", normal_distance_weight_, 0.1);
	pnh_.param("max_iterations", max_iterations_, 10000);
	pnh_.param("distance_threshold", distance_threshold_, 0.05);
    
	// ROS stuff
	//cloud_sub_ = nh_.subscribe<PointCloud>("/chewed_cloud", 1, &PoleExtractor::pointCloudCallback, this);
    	cloud_sub_.subscribe(nh_, "/chewed_cloud", 1);
	tf_filter_ = new tf::MessageFilter<PointCloud>(cloud_sub_, tf_, gripper_tag_frame_id_, 10);
    	tf_filter_->registerCallback( boost::bind(&PoleExtractor::pointCloudCallback, this, _1) );
    
	cylinder_pub_ = nh_.advertise<PointCloud>("pole_cloud", 1);
	roi_cloud_pub_ = nh_.advertise<PointCloud>("roi_cloud", 1);
    	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pole_extractor_markers", 1);
    	pole_pub_ = nh_.advertise<pole_structure_mapper::PoleSectionStamped>("pole", 10);

	control_sub_ = nh_.subscribe("/extract_poles", 10, &PoleExtractor::controlCallback, this);

	run_ = true;
}

PoleExtractor::~PoleExtractor()
{
	
}

double PoleExtractor::max(double a, double b)
{
	if(a>b) return a;
	else return b;
}

double PoleExtractor::min(double a, double b)
{
	if(a<b) return a;
	else return b;
}

void PoleExtractor::controlCallback(const std_msgs::Bool::ConstPtr& msg)
{
	run_ = msg->data;
}

void PoleExtractor::pointCloudCallback(const PointCloud::ConstPtr& msg)
{
	ROS_INFO("Pole Extractor - %s - Got a cloud msg...", __FUNCTION__);

    geometry_msgs::PointStamped corners[8];
    
    corners[0].point.x = filter_box_size_/2.0;
    corners[0].point.y = filter_box_size_/2.0;
    corners[0].point.z = filter_box_size_/2.0;
    
    corners[1].point.x = filter_box_size_/-2.0;
    corners[1].point.y = filter_box_size_/2.0;
    corners[1].point.z = filter_box_size_/2.0;
    
    corners[2].point.x = filter_box_size_/-2.0;
    corners[2].point.y = filter_box_size_/-2.0;
    corners[2].point.z = filter_box_size_/2.0;
    
    corners[3].point.x = filter_box_size_/2.0;
    corners[3].point.y = filter_box_size_/-2.0;
    corners[3].point.z = filter_box_size_/2.0;
    
    corners[4].point.x = filter_box_size_/2.0;
    corners[4].point.y = filter_box_size_/2.0;
    corners[4].point.z = filter_box_size_/-2.0;
    
    corners[5].point.x = filter_box_size_/-2.0;
    corners[5].point.y = filter_box_size_/2.0;
    corners[5].point.z = filter_box_size_/-2.0;
    
    corners[6].point.x = filter_box_size_/-2.0;
    corners[6].point.y = filter_box_size_/-2.0;
    corners[6].point.z = filter_box_size_/-2.0;
    
    corners[7].point.x = filter_box_size_/2.0;
    corners[7].point.y = filter_box_size_/-2.0;
    corners[7].point.z = filter_box_size_/-2.0;
    
    for(int i=0 ; i<8 ; i++)
    {
        corners[i].header.frame_id = gripper_tag_frame_id_;
        corners[i].header.stamp = msg->header.stamp;
        
        try { tf_.transformPoint(msg->header.frame_id, corners[i], corners[i]); }
        catch(tf::TransformException &ex) 
        {
            ROS_ERROR("Pole Extractor - %s - Error: %s", __FUNCTION__, ex.what());
            return;
        }
    }

    PointT max_p, min_p;
    max_p.x = min_p.x = corners[0].point.x;
    max_p.y = min_p.y = corners[0].point.y;
    max_p.z = min_p.z = corners[0].point.z;
    for(int i=1 ; i<8 ; i++)
    {
        if(corners[i].point.x > max_p.x) max_p.x = corners[i].point.x;
        if(corners[i].point.x < min_p.x) min_p.x = corners[i].point.x;
        
        if(corners[i].point.y > max_p.y) max_p.y = corners[i].point.y;
        if(corners[i].point.y < min_p.y) min_p.y = corners[i].point.y;
        
        if(corners[i].point.z > max_p.z) max_p.z = corners[i].point.z;
        if(corners[i].point.z < min_p.z) min_p.z = corners[i].point.z;
    }
    
	pcl::PassThrough<PointT> filter;
	filter.setInputCloud(msg);
	filter.setFilterFieldName("x");
	filter.setFilterLimits(min_p.x, max_p.x);
	filter.filter(*cloud_filtered_);
	filter.setInputCloud(cloud_filtered_);
	filter.setFilterFieldName("y");
	filter.setFilterLimits(min_p.y, max_p.y);
	filter.filter(*cloud_filtered_);
	filter.setFilterFieldName("z");
	filter.setFilterLimits(min_p.z, max_p.z);
	filter.filter(*cloud_filtered_);

	roi_cloud_pub_.publish(cloud_filtered_);
    
    /*pcl::CropBox<pcl::PointXYZ> filter;
     filter.setInputCloud(msg);
     
     Eigen::Vector4f min_pt(min.pose.position.x, min.pose.position.y, min.pose.position.z, 1);
     Eigen::Vector4f max_pt(max.pose.position.x, max.pose.position.y, max.pose.position.z, 1);
     
     filter.setMin(min_pt);
     filter.setMax(max_pt);
     
     filter.filter(*cloud_filtered_);*/

	//*cloud_filtered_ = *msg;

	//ROS_INFO("PoleExtractor - %s - Got a PointCloud with %d points, after filtering %d points remained!", __FUNCTION__, msg->points.size(), cloud_filtered_->points.size());
  	
  	got_cloud_ = true;
}

void PoleExtractor::getMinMax3DalongAxis(const PointCloud::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT * axis_point, tf::Vector3 * normal)
{
    PointT max_p = *axis_point;
    double max_t = 0.0;
    PointT min_p = *axis_point;
    double min_t = 0.0;
    
    BOOST_FOREACH(const PointT& pt, cloud->points)
    {
        // First we convert pt into a point p along the axis on the same plane perpendicular to the axis
        double t = (normal->x()*(pt.x-axis_point->x) + normal->y()*(pt.y-axis_point->y) + normal->z()*(pt.z-axis_point->z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
        
        PointT p;
        p.x = axis_point->x + normal->x() * t;
        p.y = axis_point->y + normal->y() * t;
        p.z = axis_point->z + normal->z() * t;
        
        // Now we check if the point is min or max
        if(t>max_t)
        {
            max_t = t;
            max_p = p;
        }
        if(t<min_t)
        {
            min_t = t;
            min_p = p;
        }
    }
    
    *max_pt = max_p;
    *min_pt = min_p;
}

bool PoleExtractor::cylinderSegmentation()
{
	if(!got_cloud_)
    {
	    //ROS_WARN("Pole Extractor - %s - No cloud to work on!", __FUNCTION__);
	    return false;
    }
    got_cloud_ = false;
    
    ROS_INFO("Pole Extractor - %s - Running segmentation...", __FUNCTION__);

	// Estimate point normals
	ne_.setSearchMethod(tree_);
	ne_.setInputCloud(cloud_filtered_);
	ne_.setKSearch(50);
	ne_.compute(*cloud_normals_);
	//ROS_INFO("Normals - done");

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg_.setOptimizeCoefficients(true);
	seg_.setModelType(pcl::SACMODEL_CYLINDER);
	seg_.setMethodType(pcl::SAC_RANSAC);
	seg_.setNormalDistanceWeight(normal_distance_weight_);
	seg_.setMaxIterations(max_iterations_);
	seg_.setDistanceThreshold(distance_threshold_);
	seg_.setRadiusLimits(min_diameter_, max_diameter_);
	seg_.setInputCloud(cloud_filtered_);
	seg_.setInputNormals(cloud_normals_);
	
	// Obtain the cylinder inliers and coefficients
	seg_.segment(*inliers_cylinder_, *coefficients_cylinder_);
	//ROS_INFO("Segmentation - done");

	// Extract the cylinder inliers
	extract_.setInputCloud(cloud_filtered_);
	extract_.setIndices(inliers_cylinder_);
	extract_.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>());
	extract_.filter(*cloud_cylinder);
	//ROS_INFO("Inliers - done");
	
    if(cloud_cylinder->points.size() == 0)
    {
        ROS_ERROR("Pole Extractor - %s - The cylinder point cloud is empty!", __FUNCTION__);
        return false;
    }
    
	cylinder_pub_.publish(cloud_cylinder);
    
    // Calculate the parameters of the cylinder
    PointT point_in_axis;
    point_in_axis.x = coefficients_cylinder_->values[0];
    point_in_axis.y = coefficients_cylinder_->values[1];
    point_in_axis.z = coefficients_cylinder_->values[2];
    
    // Axis vector parameters
    tf::Vector3 axis_vector(coefficients_cylinder_->values[3], coefficients_cylinder_->values[4], coefficients_cylinder_->values[5]);
    
    double cylinder_diameter = coefficients_cylinder_->values[6]*2.0;
    
    PointT top_pt, base_pt;
    getMinMax3DalongAxis(cloud_cylinder, &top_pt, &base_pt, &point_in_axis, &axis_vector);
    //ROS_INFO("MinMax3D - done");
    
    double cylinder_length = sqrt(pow(top_pt.x - base_pt.x, 2) + pow(top_pt.y - base_pt.y, 2) + pow(top_pt.z - base_pt.z, 2));
    
    //ROS_INFO("Detected a pipe section with %lfm in length and a diameter of %lfm", cylinder_length, cylinder_diameter);

    tf::Vector3 up_vector(0.0, 0.0, 1.0);
    tf::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalize();
    tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();
    geometry_msgs::Quaternion cylinder_orientation;
    tf::quaternionTFToMsg(q, cylinder_orientation);

    // Display the cylinder
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_cylinder->header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = gripper_tag_frame_id_;
    marker.id = 0;
    
    marker.type = visualization_msgs::Marker::CYLINDER;
    
    marker.action = visualization_msgs::Marker::ADD;
    
    // For the marker the position is the middle point between the bottom and the top
    marker.pose.position.x = (top_pt.x+base_pt.x)/2.0;
    marker.pose.position.y = (top_pt.y+base_pt.y)/2.0;
    marker.pose.position.z = (top_pt.z+base_pt.z)/2.0;
    marker.pose.orientation = cylinder_orientation;

    marker.scale.x = cylinder_diameter;
    marker.scale.y = cylinder_diameter;
    marker.scale.z = cylinder_length;
    
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration();
    
    // Publish the marker
    marker_pub_.publish(marker);
    
    if(run_)
    {
        // Publish the pole section
        pole_structure_mapper::PoleSectionStamped pole;
        pole.header.frame_id = cloud_cylinder->header.frame_id;
        pole.header.stamp = ros::Time::now();
        pole.pole.base.x = base_pt.x;
        pole.pole.base.y = base_pt.y;
        pole.pole.base.z = base_pt.z;
        pole.pole.axis.x = axis_vector.x();
        pole.pole.axis.y = axis_vector.y();
        pole.pole.axis.z = axis_vector.z();
        pole.pole.diameter = cylinder_diameter;
        pole.pole.length = cylinder_length;
        pole_pub_.publish(pole);
    }

    // Display roi
    visualization_msgs::Marker roi;
    roi.header.frame_id = gripper_tag_frame_id_;
    roi.header.stamp = ros::Time::now();

    roi.ns = "roi";
    roi.id = 0;
    
    roi.type = visualization_msgs::Marker::CUBE;
    
    roi.action = visualization_msgs::Marker::ADD;
    
    // For the marker the position is the middle point between the bottom and the top
    roi.pose.position.x = 0.0;
    roi.pose.position.y = 0.0;
    roi.pose.position.z = 0.0;

    roi.scale.x = filter_box_size_;
    roi.scale.y = filter_box_size_;
    roi.scale.z = filter_box_size_;
    
    roi.color.r = 0.0f;
    roi.color.g = 1.0f;
    roi.color.b = 0.0f;
    roi.color.a = 0.2;
    
    roi.lifetime = ros::Duration();
    
    // Publish the marker
    marker_pub_.publish(roi);

    ROS_INFO("Pole Extractor - %s - Done!", __FUNCTION__);
    
    return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pole_extractor_node");
    
    ROS_INFO("Pole Extractor for ROS v0.1");
	
	PoleExtractor pe;
	
	while(ros::ok())
	{
		pe.cylinderSegmentation();
        
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
}

// EOF

