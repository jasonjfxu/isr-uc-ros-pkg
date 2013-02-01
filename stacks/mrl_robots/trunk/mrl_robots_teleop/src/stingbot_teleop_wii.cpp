//turtle_teleop modified by David Portugal

#include <ros/ros.h>
//#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopStingbot{
  
public:
  TeleopStingbot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopStingbot::TeleopStingbot():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_throttle", 10, &TeleopStingbot::joyCallback, this);  //throttle topic pubs w/20Hz (wiimote uses 100Hz)
}


void TeleopStingbot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

  geometry_msgs::Twist vel;
  
  vel.linear.x = l_scale_ * joy->buttons[8]; //pra frente ou zero
  if (vel.linear.x==0.0){
    vel.linear.x = -l_scale_ * joy->buttons[9]; //pra tras ou zero
  }
  
  vel.angular.z = a_scale_ * joy->buttons[6]; //pra esq ou zero
  if (vel.angular.z==0.0){
    vel.angular.z = -a_scale_ * joy->buttons[7]; //pra dta ou zero
  }
  	
  vel_pub_.publish(vel);

}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "stingbot_teleop");
  TeleopStingbot teleop_stingbot;

  ros::spin();
}
