
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include <cereal_port/CerealPort.h>


#define MAX_ENCODER_COUNT 32768

#define AXLE_LENGTH 	0.225
#define PULSES_TO_M	0.000073529
			
#ifndef NORMALIZE
    #define NORMALIZE(z) atan2(sin(z), cos(z))	// Normalize angle to domain -pi, pi 
#endif


ros::Publisher * odom_pub_ptr;
tf::TransformBroadcaster * odom_broadcaster_ptr;

cereal::CerealPort serial_port;
ros::Time current_time, last_time;
double last_time_pub = 0.0;

bool signof (int n) { return n >= 0; }
bool confirm_communication = true;
int ID_Robot = 0;

double odometry_x_ = 0.0;
double odometry_y_ = 0.0;
double odometry_yaw_ = 0.0;
int right_encoder_prev = 0;
int left_encoder_prev = 0;


//Receive encoder ticks and send 'odom' and 'tf'
void robotDataCallback(std::string * data){ 
   
    if (confirm_communication){
      //ROS_INFO("Robot -- Communication OK! Received: \"%s\"", data->c_str());
      ROS_INFO("Stingbot is Streaming Data.");
      confirm_communication = false;
    }
    
    int first_at = data->find_first_of("@", 0);
    int second_at = data->find_first_of("@", first_at+1);    
    int first_comma = data->find_first_of(",", 0);
    int second_comma = data->find_first_of(",", first_comma+1);    

    //protection against broken msgs from the buffer (e.g., '@6,425@6,4250,6430e')
    if ( second_at > -1 || second_comma == -1){
      ROS_WARN("%s ::: ENCODER MSG IGNORED", data->c_str());
      return; 
    }    
    
    int left_encoder_count, right_encoder_count;	
    sscanf(data->c_str(), "@6,%d,%de", &right_encoder_count, &left_encoder_count);   //encoder msg parsing

    //protection against broken msgs from the buffer (e.g., '@6,425@6,4250,6430e')
    if ( abs(right_encoder_count) > MAX_ENCODER_COUNT || abs(left_encoder_count) > MAX_ENCODER_COUNT){
      ROS_WARN("Encoders > MAX_ENCODER_COUNT");
      return; 
    }
        
    double last_x = odometry_x_;
    double last_y = odometry_y_;
    double last_yaw = odometry_yaw_;           
    
    // It is not necessary to reset the encoders:    
    int right_enc_dif = 0, left_enc_dif = 0;    
    
    if ( right_encoder_prev >= (0.75*MAX_ENCODER_COUNT) && signof (right_encoder_count) == 0 && signof(right_encoder_prev) == 1 ){
      right_enc_dif = (MAX_ENCODER_COUNT-right_encoder_prev) + (MAX_ENCODER_COUNT+right_encoder_count);
          
    }else if (right_encoder_prev <= (-0.75*MAX_ENCODER_COUNT) && signof (right_encoder_count) == 1 && signof(right_encoder_prev) == 0){
      right_enc_dif = -(MAX_ENCODER_COUNT+right_encoder_prev) + (right_encoder_count-MAX_ENCODER_COUNT);
      
    }else{
      right_enc_dif = right_encoder_count - right_encoder_prev;
    }
    
    
    if ( left_encoder_prev >= (0.75*MAX_ENCODER_COUNT) && signof (left_encoder_count) == 0 && signof(left_encoder_prev) == 1){
      left_enc_dif = (MAX_ENCODER_COUNT-left_encoder_prev) + (MAX_ENCODER_COUNT+left_encoder_count);
      
    }else if (left_encoder_prev <= (-0.75*MAX_ENCODER_COUNT) && signof (left_encoder_count) == 1 && signof(left_encoder_prev) == 0){
      left_enc_dif = -(MAX_ENCODER_COUNT+left_encoder_prev) + (left_encoder_count-MAX_ENCODER_COUNT);
      
    }else{
      left_enc_dif = left_encoder_count - left_encoder_prev;
    }      
  
    //calulate Odometry: 
    double dist = (right_enc_dif*PULSES_TO_M + left_enc_dif*PULSES_TO_M) / 2.0; 
    double ang = (right_enc_dif*PULSES_TO_M - left_enc_dif*PULSES_TO_M) / AXLE_LENGTH;
    //bool publish_info = true;

    /*if (right_encoder_prev == right_encoder_count && left_encoder_prev == left_encoder_count){
	publish_info = false;
    }*/
    
    // update previous encoder counts:
    left_encoder_prev = left_encoder_count;
    right_encoder_prev = right_encoder_count;

    // Update odometry
    odometry_yaw_ = NORMALIZE(odometry_yaw_ + ang);		// rad
    odometry_x_ = odometry_x_ + dist*cos(odometry_yaw_);	// m
    odometry_y_ = odometry_y_ + dist*sin(odometry_yaw_);	// m   
    
    last_time = current_time;
    current_time = ros::Time::now();      

    // Calculate the robot speed 
    double dt = (current_time - last_time).toSec();
    double vel_x = (odometry_x_ - last_x)/dt;
    double vel_y = (odometry_y_ - last_y)/dt;
    double vel_yaw = (odometry_yaw_ - last_yaw)/dt;    

    //Publish at least at most at 75Hz
    /*if (current_time.toSec() - 0.013 >= last_time_pub){
	publish_info = true;
    }*/
    
    //if(publish_info){

	last_time_pub = current_time.toSec();
    
	// Since all odometry is 6DOF we'll need a quaternion created from yaw    
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_yaw_);

	// First, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	    
	odom_trans.transform.translation.x = odometry_x_;
	odom_trans.transform.translation.y = odometry_y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	    
	// Send the transform
	odom_broadcaster_ptr->sendTransform(odom_trans); 		// odom->base_link transform
		
	    
	// Next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	    
	// Set the position
	odom.pose.pose.position.x = odometry_x_;
	odom.pose.pose.position.y = odometry_y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	    
	// Set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vel_x;
	odom.twist.twist.linear.y = vel_y;
	odom.twist.twist.angular.z = vel_yaw;
	    
	// Publish the message
	odom_pub_ptr->publish(odom);		// odometry publisher 
    //} 
}

std::string drive(double linear_speed, double angular_speed, int ID_Robot) {
  
  //truncate to 3 decimals:
  int linear_speed_3decimals = floorf(linear_speed * 1000 + 0.5);
  int angular_speed_3decimals = floorf(angular_speed * 1000 + 0.5);
  
  std::string vel_lin_str, vel_ang_str;
  std::stringstream out_lin, out_ang;
  
  //int to string:
  out_lin << linear_speed_3decimals;
  vel_lin_str = out_lin.str();
  
  out_ang << angular_speed_3decimals;
  vel_ang_str = out_ang.str();
  
  std::string result = "@11," + vel_lin_str + "," + vel_ang_str + "e";
     
  return result;
}

//receive cmds_vel from nav_stack
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel){
  
	std::string result = drive(cmd_vel->linear.x, cmd_vel->angular.z, ID_Robot);
	ROS_INFO("[cmd_vel]: Vlinear = %f, Vangular = %f",cmd_vel->linear.x, cmd_vel->angular.z);
	ROS_INFO("Sending: %s\n", result.c_str());
	serial_port.write( result.c_str() ); 
}

int main(int argc, char** argv){ //typical usage: "./stingbot_node /dev/ttyACMx"
  
	ros::init(argc, argv, "stingbot_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port;
	
	if (argc<2){
	 port="/dev/ttyACM0";
	 ROS_WARN("No Serial Port defined, defaulting to \"%s\"",port.c_str());
	 ROS_WARN("Usage: \"rosrun [pkg] robot_node /serial_port\"");
	}else{
	  port=(std::string)argv[1];
	  ROS_INFO("Serial port: %s",port.c_str());
	}	
	
	// ROS publishers and subscribers
 	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 100);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelReceived);
    
	odom_pub_ptr = &odom_pub;
	odom_broadcaster_ptr = &odom_broadcaster;	
	
	// baud_rate and serial port:	
	int baudrate;
	pn.param<std::string>("port", port, port.c_str()); 
	pn.param("baudrate", baudrate, 115200); 

	// Open the serial port to the robot
	try{ serial_port.open((char*)port.c_str(), baudrate); }
	catch(cereal::Exception& e){
		ROS_FATAL("Robot -- Failed to open serial port!");
		ROS_BREAK();
	}
    
	//wait (2.5 seconds) until serial port gets ready
	ros::Duration(2.5).sleep(); 	
		
	// Ask Robot ID from the Arduino board (stored in the EEPROM)
	ROS_INFO("Starting Stingbot...");
	serial_port.write("@5e");
	std::string reply;

	try{ serial_port.readBetween(&reply,'@','e'); }	
	catch(cereal::TimeoutException& e){
	  ROS_ERROR("Initial Read Timeout!");
	}
	
	int VDriver, Temperature, OMNI_Firmware, Battery;
	sscanf(reply.c_str(), "@5,%d,%d,%d,%d,%de", &Temperature, &OMNI_Firmware, &Battery, &VDriver, &ID_Robot);   //encoder msg parsing

	ROS_INFO("Stingbot ID = %d", ID_Robot);
	if (ID_Robot < 4 || ID_Robot > 5){
		ROS_WARN("Attention! Unexpected Stingbot ID!");
	}
	ROS_INFO("OMNI Board Temperature = %.2f C", Temperature*0.01);
	ROS_INFO("OMNI Firmware = %.2f", OMNI_Firmware*0.01);
	ROS_INFO("Arduino Firmware Version = %d.00", VDriver);

	if (VDriver > 1500){
		ROS_ERROR("Reset Robot Connection and try again.");
		return(0);
	}

	ROS_INFO("Battery = %.2f V", Battery*0.01 );
	if (Battery < 100){
		ROS_ERROR("Robot is off. Check power source!");
		return(0);
	}
	if (Battery < 850 && Battery > 100){
		ROS_WARN("Warning! Low Battery!");
	}
	
	// Start receiving streaming data
	if( !serial_port.startReadBetweenStream(boost::bind(&robotDataCallback, _1), '@', 'e') ){
		  ROS_FATAL("Robot -- Failed to start streaming data!");
		  ROS_BREAK();
	}
	serial_port.write("@18e");
    
	ros::spin(); //trigger callbacks and prevents exiting
  	return(0);
}
