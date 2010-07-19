#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "roomba_tf_setup");
  	ros::NodeHandle n;

  	ros::Rate r(100);

  	tf::TransformBroadcaster broadcaster;

  	while(n.ok())
	{
    		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.065, 0.000, 0.240)),
        	ros::Time::now(),"base_link", "base_laser"));
		
    		r.sleep();
  	}
}
