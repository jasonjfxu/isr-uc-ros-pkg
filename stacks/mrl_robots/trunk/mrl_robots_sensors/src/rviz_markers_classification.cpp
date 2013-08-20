#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

float x=0, y=0, z=0, w=0;
float x_temp=0, y_temp=0, z_temp=0, w_temp=0;
float dust_sensor=0, acohol_sensor=0, thermopile_sensor=0;
int cnt=0;

ros::Publisher marker_pub;
ros::Subscriber classification_sub;
ros::Subscriber dataSensors_sub;
ros::Subscriber odom_sub;
ros::Subscriber amcl_sub;

// void classificationCallback(std::string * data) {
//   
// }

void dataSensorsCallback(const geometry_msgs::Vector3::ConstPtr& msg) {

  acohol_sensor=msg->x;
  dust_sensor=msg->y;
  thermopile_sensor=msg->z;

}


//void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  
  x=msg->pose.pose.position.x;
  y=msg->pose.pose.position.y;
  z=msg->pose.pose.orientation.z;
  w=msg->pose.pose.orientation.w;
  
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  //shape = visualization_msgs::Marker::SPHERE;
  //shape = visualization_msgs::Marker::ARROW;
  //shape = visualization_msgs::Marker::CYLINDER;
  //shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one

  marker.ns = "basic_shapes";
  marker.id = cnt;	
  cnt++;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;

  marker.scale.x = 0.20;
  marker.scale.y = 0.23;
  marker.scale.z = 0.3;

  if((thermopile_sensor>40) && (thermopile_sensor<145)){
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.02;
  } else if ((thermopile_sensor>145) && (thermopile_sensor<160)) {
    
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.08;
  } else if (thermopile_sensor>160) {
    
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.3;
  } else {
    
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.02;
  }
  
   
 if((acohol_sensor>360) && (acohol_sensor<390)){
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.02;
  } else if ((acohol_sensor>390) && (acohol_sensor<410)) {
    
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.08;
  } else if (acohol_sensor>410) {
    
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.3;
  } else if (thermopile_sensor<40){
    
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.02;
  }
 
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "classification_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_robot", 1);
  odom_sub = n.subscribe("odom", 1, odomCallback);
  //classification_sub = n.subscribe("sensors_classiffication", 1, classificationCallback);
  dataSensors_sub = n.subscribe("robot_sensors", 1, dataSensorsCallback);

  ros::spin();
  
}
