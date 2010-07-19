#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float posX, posY, angleYaw;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navi_goals");


	// get (x, y, theta) from command line	
	if(argc > 1 && argc <4)
	{
		for(int i = 1;i < argc;i++)
			printf("argv[%d]:%s\n",i, argv[i]);
	}


	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	if(argc == 2)
	{
		posX = atof(argv[1]);
		posY = 0.0;
		angleYaw = 0.0;
	}
	else if(argc == 3)
	{
		posX = atof(argv[1]);
		posY = atof(argv[2]);
		angleYaw = 0.0;
		//angleYaw = atan2(posY,posX);
		
	}
	else
	{
		posX = 1.0;
		posY = 0.0;
		angleYaw = 0.0;
	}
	
	// quaternion created from angleYaw
    	geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angleYaw);
		
	// send a goal to the robot
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	ROS_INFO("getting (x:%f,y:%f) - (x:%f,y:%f,z:%f,w:%f)",posX, posY,angle_quat.x,angle_quat.y,angle_quat.z,angle_quat.w);

	goal.target_pose.pose.position.x = posX;
	goal.target_pose.pose.position.y = posY;
	goal.target_pose.pose.orientation = angle_quat;
//	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved forward");
	else
		ROS_INFO("The base failed to move forward for some reason");

	return 0;
}
