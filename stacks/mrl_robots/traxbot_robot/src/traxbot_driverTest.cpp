/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
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
* Author: André Araújo, 2012
*********************************************************************/


//////////////////////////////////////////////////////////////
//		TRAXBOT ROS DRIVE TESTS
//////////////////////////////////////////////////////////////


// STD LIB
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>



// ROS LIB
#include <ros/ros.h>				// Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.1415

float info_battery, info_driverF;
int info_temp, range_msg1, range_msg2, range_msg3, encoderRead1, encoderRead2;

int x=0, y=0, xTemp=0 , yTemp=0;
float teta=0, tetaTemp=0;                       // global pose

int xFinal=0, yFinal=0;
float tetaFinal=0;                      // final pose

float dist=0,alfa=0,phi=0;          // auxiliary variables of final pose

float r_wheel=39.0, r_robot=185.0;   // robot's dimensions in [mm]

int vMove = 25,  vRotate = 20;       // set speeds 


int reactiveRange = 35;			 // range sonar limit 

int samples = 10;
float sonar1Samples[10], sonar2Samples[10], sonar3Samples[10]; 
int counter1 = 0, counter2 = 0, counter3 = 0;

/* Define publisher topics */

// TraxBot INFO
ros::Publisher pub_traxbotInfo;
ros::Publisher pub_sonars;


// TraxBot MOTORS
ros::Publisher pub_moveMotors; 
ros::Publisher pub_stopMotors;
ros::Publisher pub_encodersReset; 
ros::Publisher pub_encodersRead;

// Driver
ros::Publisher pub_odometry;


/* Define subscriber topics */

// TraxBot INFO  
ros::Subscriber sub_driverFirmware;
ros::Subscriber sub_batteryPower;
ros::Subscriber sub_driverTemperature;

// TraxBot MOTORS
ros::Subscriber sub_encoderRead1;
ros::Subscriber sub_encoderRead2;

// SONARS
ros::Subscriber sub_range1;
ros::Subscriber sub_range2;
ros::Subscriber sub_range3;

 
float median(int n, float x[]) {			// Median function to filter sonars reads  (n samples, x[] tab of values)
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
 
    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[n/2];
    }
}


void enable_encodersRead() {	// Publish an empty msg to enable encoder 1 and 2 readings

  std_msgs::Empty enable1;
  pub_encodersRead.publish(enable1);
  ros::spinOnce();
//   ROS_INFO("ENVIEI MSG ENCODERS");
}


void enable_encodersReset() {	// Publish an empty msg to enable reset encoders
  
  std_msgs::Empty enable2;
  pub_encodersReset.publish(enable2);
  ros::spinOnce();
  printf("ENCODERS RESET\n");
}


void enable_stopMotors() {	// Publish an empty msg to enable stop motors
  
  std_msgs::Empty enable3;
  pub_stopMotors.publish(enable3);
  ros::spinOnce();
  printf("STOP MOTORS\n");

}

void enable_infoData() {	// Publish an empty msg to enable arduino to send info data
  
  std_msgs::Empty enable4;
  pub_traxbotInfo.publish(enable4);
  ros::spinOnce();
}



void encoder1_readCB(const std_msgs::Int16::ConstPtr& pulses1) {		// Read subscribed msg of encoder 1 pulses
  
  encoderRead1 = pulses1->data; 
//   ROS_INFO("ENCODER 1 CALLBACK: [%d]", encoderRead1);
}

void encoder2_readCB(const std_msgs::Int16::ConstPtr& pulses2) {		// Read subscribed msg of encoder 2 pulses
  
  encoderRead2 = pulses2->data;
//   ROS_INFO("ENCODER 2 CALLBACK: [%d]", encoderRead2);
}

void batteryPowerCB(const std_msgs::Float32::ConstPtr& battery) {	// Read subscribed msg of battery info

  info_battery = battery->data;
}

void firmwareVersionCB(const std_msgs::Float32::ConstPtr& firmwareV) {	// Read subscribed msg of Omni3MD firmware version

  info_driverF = firmwareV->data;
}

void temperatureCB(const std_msgs::Int8::ConstPtr& temp) {	// Read subscribed msg of temperature info

  info_temp = temp->data;
}

void sonar1CB(const std_msgs::Int16::ConstPtr& sonar1) {	// Read subscribed msg of sonar 1 range

  range_msg1 = sonar1->data;
  
  sonar1Samples[counter1] = sonar1->data;		// Filter values
 
  if (counter1 == 0) {
   
   range_msg1 = sonar1Samples[0];
  
  }
  
  counter1++;
  
  if ( counter1 >= samples) {
      counter1=0;
      range_msg1 = int(floor(median(samples, sonar1Samples)));
  }
  
}

void sonar2CB(const std_msgs::Int16::ConstPtr& sonar2) {	// Read subscribed msg of sonar 2 range

  range_msg2 = sonar2->data;
  
  sonar2Samples[counter2] = sonar2->data;		// Filter values
 
  if (counter2 == 0) {
   
   range_msg2 = sonar2Samples[0];
  }
  
  counter2++;
  
  if ( counter2 >= samples) {
      counter2=0;
      range_msg2 = int(floor(median(samples, sonar2Samples)));
  }
  
}

void sonar3CB(const std_msgs::Int16::ConstPtr& sonar3) {	// Read subscribed msg of sonar 3 range

  range_msg3 = sonar3->data;
  
  
  sonar3Samples[counter3] = abs(sonar3->data);		// Filter values
 
  if (counter3 == 0) {
   
   range_msg3 = sonar3Samples[0];
  }
  
  counter3++;
  
  if ( counter3 >= samples) {
      counter3 = 0;
    range_msg3 = int(floor(median(samples, sonar3Samples)));
  }
  
  //ROS_INFO("Sonar 3: [%d]", range_msg3);
}


void moveMotors(int speed1, int speed2, int way) {			// Move motors
  
  geometry_msgs::Point32 move;
  
  move.x = speed1;		// set speed motor 1   0 to 100
  move.y = speed2;		// set speed motor 2   0 to 100
  move.z = way;			// 0= linear move,  1= rotate CCW (counterclockwise)   2= rotate CW (clockwise)
  
  pub_moveMotors.publish(move);
  ros::spinOnce();
  printf("START MOTORS\n");
 
}

bool powerCheck( float batteryValue) {
  bool check;

  if (batteryValue<=2) {
    
    check=0;
  } else {
    check=1;
  }
  
 return check;
   
}

float pulse2dist (int pulse) {		// Pulse to cm
 
 return ((pulse*1000)/11600); //11270 3cm  //11500 1cm //robot2
  
}


int encodersRead(){
  
  enable_encodersRead();
  ros::spinOnce();
  int enco1 = abs(encoderRead1);
  int enco2 = abs(encoderRead2);
  int enco=round((enco1+enco2)/2.0);
  
  return enco;
}


void calcOdometryDist(float dist, float alfa) {
  
  geometry_msgs::Pose2D odm;
  float D = dist;
  
  teta = tetaTemp + alfa;
        
  if (teta < -180) {
     teta+=360;
  }else if (teta > 180){
     teta-=360; 	
  }

  x = xTemp + round(D*cos(teta * (PI/180)));
  y = yTemp + round(D*sin(teta * (PI/180)));
	
  tetaTemp = teta;
  xTemp = x;
  yTemp = y;
  
  odm.x = x;
  odm.y = y;
  odm.theta = teta;
  
  pub_odometry.publish(odm);
  ros::spinOnce();

  printf("GLOBAL ODOMETRY UPDATED x:%d y:%d theta:%f\n",x,y,teta);
}


void move(float dist, int speed1, int speed2)
{
  long poss1=0, poss_dist=0, poss1_buf=0;
  int cnt = 0;
  
  printf("START MOVE()\n");
  
  poss_dist = dist;
  printf("Goal position %ld mm\n", poss_dist);
 
  moveMotors(speed1, speed2, 0);
  
  
  while(poss1 < poss_dist){                               // Start loop to drive motors forward

    usleep(100000);					  // Very important: delay time necessary  to dont congest msgs

    poss1 = pulse2dist ( encodersRead() );
    printf("Actual distance: %ld\n",poss1);
    
    calcOdometryDist((poss1-poss1_buf), 0);
    poss1_buf = poss1;
  }
  
  printf("ARRIVE TO GOAL with more %d mm.\n", (poss1-poss_dist));
  
  usleep(100000);
  enable_stopMotors();
  ros::spinOnce();

  usleep(100000);
  enable_encodersReset();
  ros::spinOnce();
  
  while (cnt<5) {
    usleep(100000);
    enable_encodersRead();
    ros::spinOnce();
    ++cnt;
  }
  
  printf("FINISH MOVE()\n");
 
} // END move()


void rotate(float angle, int speed){
  
  int poss_dist=0, cnt=0, direction=1;
  float ang = 0, ang_buf=0;
  
  printf("START ROTATE()\n");
   
  if (angle != 0) {    

    poss_dist=((1750.0*angle)/360.0)*(r_robot/r_wheel);   
    
    printf("Goal position %d pulses. %f degrees\n", poss_dist,angle);
    
  
    while(abs(ang) <= abs(angle)){                       // Start loop to turn motors
      ++cnt;
      usleep(100000);					  // Very important: delay time necessary  to dont congest msgs
      
      ang=(r_wheel/r_robot)*(poss_dist/abs(poss_dist))*encodersRead()*360.0/1750.0;
      
      printf("Actual angle: %f\n", ang);
      
       if (cnt<2) {
      
	if (poss_dist>0 ) {
	  moveMotors(speed,speed,1);
	  printf("Turn Left\n");
	} else {
	  moveMotors(speed,speed,2);
	  printf("Turn Right\n");
	}
      }
      
      calcOdometryDist(0.0, (ang-ang_buf));		// Update odometry
      ang_buf = ang;
      
    } // end while
  
  } // end 1st if
  
  if (ang <0 && ang!=0) {			// Compensar VER NOS OUTROS ROBOS
      ang=ang+5;
      calcOdometryDist(0.0, (ang-ang_buf));	
  }
  
  if (ang >0 && ang!=0) {
      ang=ang-5;
      calcOdometryDist(0.0, (ang-ang_buf));
   }
  // calcOdometryDist(0.0, ang);
  printf("ARRIVE TO GOAL with more %f degrees\n", (ang-angle));
  
  usleep(100000);
  enable_stopMotors();
  ros::spinOnce();
  
  
  usleep(100000);
  enable_encodersReset();
  ros::spinOnce();
  
  cnt=0;
  while (cnt<5) {
    usleep(100000);
    enable_encodersRead();
    ros::spinOnce();
    ++cnt;
  }
  
  printf("FINISH ROTATE()\n");
}


void goTo(int x, int y, float teta, int xFinal, int yFinal, float tetaFinal){
   
  printf("START GO TO()\n");
  
  dist=sqrt(pow(xFinal-x,2)+pow(yFinal-y,2));
  alfa=atan2((yFinal-y),(xFinal-x))*180.0/PI;
  phi=alfa-teta;
              
  //1- ROTATE INTO THE FINAL POINT DIRECTION
  if (abs(phi)>180)
  {
   phi=phi-(abs(phi)/phi)*360.0;
  }
                  
                  
  printf("PHI ROTATION: %d \n", phi);
  rotate(phi,vRotate);

  //2- MOVE IN A STRAIGHT LINE
  printf("LINEAR MOVE: %d \n", dist);
  move(dist,vMove,vMove);

                
  //3- ROTATE TO FINAL POSE
  phi=tetaFinal-alfa;
                 	  
  if (tetaFinal==0){ tetaFinal=190;}
                 
  if ((tetaFinal>-180) && (tetaFinal<=180))  {
    printf("FINAL ROTATION: %d \n", phi);
    rotate(phi,vRotate);
    teta=tetaFinal;
        
  }else{
      teta=alfa;
  }
  
  printf("FINISH GO TO()\n");	   		
}


void squareTest(int side_width, int way, int cycles) {
  
  printf("START SQUARE TEST()\n");
  
  int cnt = 0;
  
  while (cnt < cycles) {
    
    switch (way) {	// way=0  CW (clockwise)      way=1  CCW (counterclockwise) 
    
      case 0:
  
        goTo(x,y,teta,0,side_width,0);
	goTo(x,y,teta,side_width,side_width,0);
	goTo(x,y,teta,side_width,0,0);
	goTo(x,y,teta,0,0,0);
	

	break;
    
      case 1:
	goTo(x,y,teta,side_width,0,0);
	goTo(x,y,teta,side_width,side_width,0);
	goTo(x,y,teta,0,side_width,0);
	goTo(x,y,teta,0,0,0);
	  
	break;
      
      default:
	break;
    }
    ++cnt;
  }
  
  
  printf("FINISH SQUARE TEST()\n");
  
}


void reactiveNavigation() {
  
  int cnt = 0;
  
  while(cnt < 30000){                               // Start loop to drive motors forward

   moveMotors(vMove, vMove, 0);
   usleep(10000);					  // Very important: delay time necessary  to dont congest msgs
   ros::spinOnce();	//IMPORTANT	


    ROS_INFO("Moving... Sonar1: [%d], Sonar2: [%d], Sonar3: [%d]\n",range_msg1,range_msg2,range_msg3);
    
    while (range_msg1 <= reactiveRange || range_msg2 <= reactiveRange || range_msg3 <= reactiveRange ) {
      

      ROS_INFO("Obstacle detected... Sonar1: [%d], Sonar2: [%d], Sonar3: [%d]\n",range_msg1,range_msg2,range_msg3);
      ros::spinOnce();
      
      
      if (range_msg3 <= range_msg2) {
	
	 
		while (range_msg1<=reactiveRange || range_msg3<=reactiveRange) {
			usleep(10000);
			rotate(10,vRotate);

			ROS_INFO("ROTATION LEFT... Sonar1: [%d], Sonar2: [%d], Sonar3: [%d]\n",range_msg1,range_msg2,range_msg3);
			ros::spinOnce();
		}
 
 	  
		} else  {
 
		while (range_msg1<=reactiveRange || range_msg2<=reactiveRange) {
		      usleep(10000);
		      rotate(-10,vRotate);
		      ROS_INFO("ROTATION RIGHT... Sonar1: [%d], Sonar2: [%d], Sonar3: [%d]\n",range_msg1,range_msg2,range_msg3);
		      ros::spinOnce();
		}
      }
      
     }
    ++cnt;
  }
  
  usleep(100000);
  enable_stopMotors();
  ros::spinOnce();

  usleep(100000);
  enable_encodersReset();
  ros::spinOnce();
  
}


void stepTest() {
	
	goTo(x,y,teta,400,0,0);
	goTo(x,y,teta,400,400,0);
	goTo(x,y,teta,800,400,0);
	goTo(x,y,teta,800,800,0);	
	goTo(x,y,teta,1200,800,0);
	goTo(x,y,teta,1200,1200,0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Traxbot_DriverTest");
  ros::NodeHandle nh;

  ROS_INFO("ROS in TraxBot: Start ROS driver test...");    // ROS_INFO(), replacement for printf/cout   
   
  
  /* Subscribe topics */
  sub_range1 = nh.subscribe("/sonar_traxbot_front", 1, sonar1CB);  
  sub_range2 = nh.subscribe("/sonar_traxbot_right", 1, sonar2CB);  
  sub_range3 = nh.subscribe("/sonar_traxbot_left", 1, sonar3CB);  
  sub_encoderRead1 = nh.subscribe("/encoderRead1", 1, encoder1_readCB);  
  sub_encoderRead2 = nh.subscribe("/encoderRead2", 1, encoder2_readCB);  
  sub_batteryPower = nh.subscribe("/batteryPower", 1, batteryPowerCB);  
  sub_driverTemperature = nh.subscribe("/driverTemperature", 1, temperatureCB);  
  sub_driverFirmware = nh.subscribe("/driverFirmware", 1, firmwareVersionCB);  

  /* Advertise topics */
  pub_moveMotors = nh.advertise<geometry_msgs::Point32>("/moveMotors", 1);
  pub_traxbotInfo = nh.advertise<std_msgs::Empty>("/traxbotInfo", 1);
  pub_sonars = nh.advertise<std_msgs::Empty>("/sonars", 1);
  pub_stopMotors = nh.advertise<std_msgs::Empty>("/stopMotors", 1);
  pub_encodersReset = nh.advertise<std_msgs::Empty>("/encodersReset", 1);
  pub_encodersRead = nh.advertise<std_msgs::Empty>("/encodersRead", 1);
  pub_odometry = nh.advertise<geometry_msgs::Pose2D>("/odometry", 1);

  ros::Rate loop_rate(10); //0.01 segundos 
  
  int count=0;
 
  
  
  while( ros::ok()){
    
    
    if (count <= 1) {		
      enable_encodersReset();
      enable_stopMotors();
    }

    if (count == 150) {
      

//	move(distance, vMove, vMove);
//	rotate(angle, vRotate);
//      goTo(int x, int y, float teta, int xFinal, int yFinal, float tetaFinal)
//      squareTest(1000, 0, 1);
//      reactiveNavigation();
//      stepTest();
      
    }
    
    printf("count: %d\n",count); 
    ++count;
    ros::spinOnce();
    loop_rate.sleep();
    
  }
   


    ROS_INFO("DONE");

}
