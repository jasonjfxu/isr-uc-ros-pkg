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



// STD LIB
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
//#include <time.h>
//#include "traxbot/traxbot.h"


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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265

#define sonarSamples 4
#define maxRange 1.5
#define rotateSpeed 0.4
#define moveSpeed 0.3
#define reactiveRange 0.5

#define navigation REACTIVE		// COGNITIVE

ros::Publisher cmd_velocidade;

ros::Subscriber odm_stage;
ros::Subscriber sonarsRange_stage;


sensor_msgs::LaserScan sonarsRange;
// nav_msgs::Odometry odometry;

// Global odometry from stage
float leftSonar, rightSonar, centerSonar;
float x=0, y=0;		// Global position
double theta=0;		// Global orientation
double thetaPol=0;	// Global angle polarization in stage

// Global odometry from goTo()
float x_goTo=0, y_goTo=0, theta_goTo=0;		// Global position



void odom_readCB(const nav_msgs::Odometry::ConstPtr& odom) {	
  
  x = odom->pose.pose.position.x;
  y = odom->pose.pose.position.y;
  theta = odom->pose.pose.orientation.w;       // This represents an orientation in free space in quaternion form.  alfa=2*(acos(w)) in degrees
  thetaPol = odom->pose.pose.orientation.z;       // Theta polarization


//   printf("theta real %f\n", theta);
  
}


float minValue(int ini_idx, int final_idx, float tab[], float maxValue ) { 	 // return min value from each samples array range from sonar
  
  int idx;
  float min= tab[ini_idx];

  for (idx=ini_idx; idx < final_idx; idx++) {
    
    if (tab[idx] < maxValue && tab[idx] < min) {
      
       min = tab[idx];
    }
  }
  return min;
}



void sonars_readCB(const sensor_msgs::LaserScan::ConstPtr& range) {		// Read 4 samples from laser, and calculate min read value
  
  float tab[3*sonarSamples];
  
  for (int i=0; i<3*sonarSamples; i++){
    tab[i]=range->ranges[i];
  }
  
  rightSonar = minValue(0, sonarSamples, tab, maxRange);
  centerSonar = minValue(sonarSamples, 2*sonarSamples, tab, maxRange);
  leftSonar = minValue(2*sonarSamples, 3*sonarSamples, tab, maxRange);
  
//   printf("right %f, center %f, left %f \n", rightSonar, centerSonar, leftSonar );

}


double quaternion2degrees(double angle, double polarization) {		// convert quaternion value to degrees, 
									// angle in quaternion value in nav_msgs::Odometry at pose.pose.orientation.w 
									// angle polarization in nav_msgs::Odometry at pose.pose.orientation.z 
									
  double angle_degrees;
  int fact=0;
  
  angle_degrees=2*( (acos (angle) * 180) / PI );
  
  if (polarization < 0) {
    fact=-1;
  } else {
    fact=1;
  }
  
  return angle_degrees=fact*angle_degrees;
}



void move(float distance, float speed){	// Stage linear move speed  0.1 to 0.5, distance in meters
  
   printf("START MOVE()\n");
   printf("Goal distance %f\n", distance);
   
   float calDist, x0 , y0;
   
   x0= x;
   y0= y;
   calDist= 0;
   
   geometry_msgs::Twist speedM;
   
   speedM.linear.x= speed;
   
   distance=distance-(moveSpeed/10); // offset
   
   while( calDist < distance ){                       // Start loop to turn motors
     
      calDist=sqrt( pow(x-x0,2) + pow(y-y0,2) );

      cmd_velocidade.publish(speedM);
      ros::spinOnce(); 

      printf("Actual distance %f\n", calDist);
  }
  
  // Stop
  speedM.linear.x= 0;
  cmd_velocidade.publish(speedM);
  ros::spinOnce(); 
  
  printf("FINISH MOVE()\n");
}



void rotate(float angle_deg, float speed){	// Stage rotation angle in degres between 180 to -180 , speed  0.1 to 0.5
  
   printf("START ROTATE()\n");
   
   double ang=0;
   float finalAng=0;
   int direction=0;

 //  printf("Rotate %f degrees. Goal position %f degrees\n", angle_deg, finalAng);	////////////////////////////////////////////////
   
   
   geometry_msgs::Twist speedR;
   
   if (angle_deg < 0 ){
     angle_deg=angle_deg+(rotateSpeed*10); // offset because the time to stop in  while loop
     speedR.angular.z= -speed;
     direction = -1;
     printf("Rotate to right.");
   } else {
     angle_deg=angle_deg-(rotateSpeed*10); // offset because the time to stop in  while loop
     speedR.angular.z= speed;
     direction = 1;
     printf("Rotate to left.");
   }
   
   
   finalAng = quaternion2degrees( theta, thetaPol ) + angle_deg;
   
   if ( finalAng > 180 ) {		// only accept values between -180 and 180
     
     finalAng = finalAng - 360;
   
    } else if ( finalAng < (-180) ) { 
      
      finalAng = finalAng + 360;
    }
   
   
   ang = quaternion2degrees( theta , thetaPol);		// Update actual angle 
   

   
////////////////////////////    POSITIVE ROTATION   
  if( direction > 0 ) {
    
    if ( ang > 0 && finalAng < 0 ) {			// In case of trasition 180 to -180
      
      while (ang > finalAng){
	cmd_velocidade.publish(speedR);
	ros::spinOnce();
	ang= quaternion2degrees( theta , thetaPol);
	printf("Actual angle %f\n", ang);
	printf("final ang %f\n", finalAng);
       
	if (ang < 0 ) {					// In transition when go from positive to negavite: (ang > finalAng) -> (ang < finalAng)
	 
	  while (ang < finalAng){
	      cmd_velocidade.publish(speedR);
	      ros::spinOnce();
	      ang= quaternion2degrees( theta , thetaPol);
	      printf("Actual angle %f\n", ang);
	      printf("final ang %f\n", finalAng);
	  }
	 break;
	 }
       }
      
      } else {		  // Rotate to reach is goal 

	while( ang < finalAng ){                       
	  cmd_velocidade.publish(speedR);
	  ros::spinOnce(); 
	  ang= quaternion2degrees( theta , thetaPol);
	  printf("Actual angle %f\n", ang);
	  printf("final ang %f\n", finalAng);
	}
      }
    
  } else {
  
////////////////////////////   NEGATIVE ROTATION   
  
   if ( ang < 0 && finalAng > 0 ) {			// In case of trasition 180 to -180

     while (ang < finalAng){
       cmd_velocidade.publish(speedR);
       ros::spinOnce();
       ang= quaternion2degrees( theta , thetaPol);
       printf("Actual angle %f\n", ang);
       printf("final ang %f\n", finalAng);
     }
      
    } else {						// Rotate to reach is goal 
              
      while( ang > finalAng ){                       
	cmd_velocidade.publish(speedR);
	ros::spinOnce(); 
	ang= quaternion2degrees( theta , thetaPol);
	printf("Actual angle %f\n", ang);
	printf("final ang %f\n", finalAng);
       }
    }  
  
}

  // Stop rotation
  speedR.angular.z= 0;
  cmd_velocidade.publish(speedR);
  ros::spinOnce(); 
  
  printf("FINISH ROTATE()\n");
}


void reactiveNavigation() {

  int cnt=0;
  
 geometry_msgs::Twist speedM;
 
 speedM.linear.x= moveSpeed ;
 
  while (cnt < 1000) {

      usleep(100000);

      printf("Moving....\n");
      cmd_velocidade.publish(speedM);
      ros::spinOnce();
      
      while (rightSonar <= reactiveRange || leftSonar<=reactiveRange || centerSonar<=reactiveRange) {		// Stop and rotate
	
        usleep(100000);
       
	//rotate(15,rotateSpeed);
	
	
	
	if (rightSonar <= leftSonar) {	

	  while (centerSonar<=reactiveRange || rightSonar<=reactiveRange) {
	    usleep(100000);
	    rotate(10,rotateSpeed);
 
	  }

	  
	} else  {

	  while (centerSonar<=reactiveRange || leftSonar<=reactiveRange) {
	    usleep(100000);
	    rotate(-10,rotateSpeed);
	  }
	}

	printf("STOP reaction....\n");
      } // end while
	
	
	
      ++cnt;
    
  } // end main while
    

}



//void goTo(float xIni, float yIni, float teta, float xFinal, float yFinal){ 			// GoTo modificado para movimentos continuos, ou seja a pose final mantem se para o 
void goTo(float xIni, float yIni, float teta, float xFinal, float yFinal, float tetaFinal){	// para o ponto seguinte. Declarar globalmente x_goTo=0 ,y_goTo=0, theta_goTo=0; 
												// tetaFinal só é ajustado quando x e y iguais
  
  float dist=0, alfa=0, phi=0;
  
  printf("START GO TO()\n");
  
  
  
  if ( xIni==xFinal && yIni==yFinal) { 		//  Quando está na mesma posição x y, está somente a rodar, ATENÇÃO não é pose final do movimento

      phi = tetaFinal - teta;
      rotate(phi,rotateSpeed);
    
     
  } else {
  
  dist=sqrt(pow(xFinal-xIni,2)+pow(yFinal-yIni,2));
  alfa=atan2((yFinal-yIni),(xFinal-xIni))*180.0/PI;
  phi=alfa-teta;
  
  //printf("dist:%f alfa:%f phi:%f \n", dist, alfa, phi);
              
  //1- ROTATE INTO THE FINAL POINT DIRECTION
  if (abs(phi)>180)
  {
   phi=phi-(abs(phi)/phi)*360.0;
  }
  
  printf("PHI ROTATION: %f \n", phi);
  rotate(phi,rotateSpeed);

  //2- MOVE IN A STRAIGHT LINE
  printf("LINEAR MOVE: %f \n", dist);
  move(dist,moveSpeed);

  }

  
  // Pose update
  x_goTo = xFinal;
  y_goTo = yFinal;
  theta_goTo = theta_goTo + phi;

  
  printf("PHI FINAL: %f \n", theta_goTo);
  printf("FINISH GO TO()\n");	   		
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Virtual_TraxBot_Stage");
  ros::NodeHandle nh;

  ROS_INFO("ROS in TraxBot: Start ROS move in stage...");    // ROS_INFO(), replacement for printf/cout   
   
  
  /* Subscribe topics */
  sonarsRange_stage = nh.subscribe("/base_scan", 1, sonars_readCB);
  odm_stage = nh.subscribe("/odom", 1, odom_readCB);
  
  /* Advertise topics */
  cmd_velocidade = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  ros::Rate loop_rate(10); //0.01 segundos 
  
  int count=0;
 
  
  
  while( ros::ok()){
  
    
    if (count == 10) {
      
      reactiveNavigation();

      
      
    }
  
    printf("count: %d\n",count);
    ++count;
    ros::spinOnce();
    loop_rate.sleep();
    
  }
   


    ROS_INFO("DONE");

}
