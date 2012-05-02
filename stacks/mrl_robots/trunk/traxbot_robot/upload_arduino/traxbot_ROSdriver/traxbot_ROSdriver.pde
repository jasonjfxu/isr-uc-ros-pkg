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
 ********************************************************************
 *
 * 
 *                  TraxBot - Arduino ROS
 *    
 *                       ROS DRIVER
 *  
 ********************************************************************/


// ROS lib
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point32.h>


// Standard lib
#include <stdio.h>
#include <string.h>
#include <math.h>

// OMNI lib
#include <Wire.h>                   // required by Omni3MD.cpp
#include <Omni3MD.h>

Omni3MD traxbot;                    // [byte omniAddress]

#define ROBOT 2        		    // Define Robot number  1, 2 or 3  (because they have slightly different parameters) 
#define OMNI3MD_ADDRESS 0x30        // default factory address 
#define FRONT_SONAR 0
#define LEFT_SONAR  1
#define RIGHT_SONAR 2
#define Kp 1300                    // PID Gain
#define Ki 350                     // PID Gain
#define Kd 400                     // PID Gain

unsigned short int vR_1e, vR_2e, vR_1d, vR_2d, vM_1, vM_2;         // Speed offset for each robot

ros::NodeHandle  nh;
std_msgs::Float32 info_power, info_driverF;
std_msgs::Int8 info_temp, robot_ID;
std_msgs::Int16 encoderRead1, encoderRead2, range_msg1, range_msg2, range_msg3;

/***********************  Define publisher topics  ***********************/

// SONARS
ros::Publisher pub_range1( "/sonar_traxbot_front", &range_msg1);
ros::Publisher pub_range2( "/sonar_traxbot_right", &range_msg2);
ros::Publisher pub_range3( "/sonar_traxbot_left", &range_msg3);

// INFO
ros::Publisher pub_batteryPower( "/batteryPower", &info_power);
ros::Publisher pub_driverTemperature( "/driverTemperature", &info_temp);
ros::Publisher pub_driverFirmware( "/driverFirmware", &info_driverF);
ros::Publisher pub_robotID( "/robotID", &robot_ID);

// MOTORS
ros::Publisher pub_encoderRead1( "/encoderRead1", &encoderRead1);
ros::Publisher pub_encoderRead2( "/encoderRead2", &encoderRead2);


/**************************  Functions  *********************************/

// SONARS
int getRange(int pin_num) {

 
  int aux_sensorValue=0, sensorValue=0, cnt=0;
  int samples = 20;

  delay(10);

  while (cnt != samples) {
     digitalWrite(12, HIGH);					// Activate chain reading on sonars
     sensorValue = analogRead(pin_num);
     sensorValue = 1.767 * pow(sensorValue,0.9465) - 0.3759;
     aux_sensorValue = aux_sensorValue + sensorValue;
     cnt = cnt + 1;
     digitalWrite(12, LOW);
   }
   cnt=0;
   sensorValue = round( aux_sensorValue / samples); 
  
   return (sensorValue);
   
}


// INFO
void infoCb (const std_msgs::Empty& msg1) {
  
  // Temperature
  float temperatureValue = traxbot.read_temperature();
  info_temp.data = (int)temperatureValue;
  pub_driverTemperature.publish(&info_temp);
  
  // Battery
  float batteryValue = traxbot.read_battery();
  info_power.data= batteryValue;
  pub_batteryPower.publish(&info_power);
  
  // Driver Firmware Version
  info_driverF.data = traxbot.read_firmware();
  pub_driverFirmware.publish(&info_driverF);
  
  // Robot ID
  robot_ID.data = ROBOT;
  pub_robotID.publish(&info_driverF);
}

void readSonars () {  
  // Sonars
  delay(10);
  range_msg1.data = getRange(FRONT_SONAR);
  pub_range1.publish(&range_msg1);

  delay(10);
  range_msg2.data = getRange(RIGHT_SONAR);
  pub_range2.publish(&range_msg2);
  
  delay(10);
  range_msg3.data = getRange(LEFT_SONAR);    
  pub_range3.publish(&range_msg3);  
  
}

// MOTORS
void moveMotorsCb(const geometry_msgs::Point32& move) {

  int vMove_1= move.x;   
  int vMove_2= move.y;
  int action= move.z;    // 0= linear move,  1= rotate (+degrees)  2= rotate (-degrees)
  
  switch (action)
  {
  case 0:
    vMove_1= vMove_1 + vM_1;   
    vMove_2= vMove_2 + vM_2;
    break;
 
 case 1:
    vMove_1= vMove_1 + vR_1e;  
    vMove_2= vMove_2 + vR_2e;
    break;
 
 case 2:
    vMove_1= vMove_1 + vR_1d;  
    vMove_2= vMove_2 + vR_2d; 
    break;
 
 default:
    break;
 }

 
  if(action==0) {
    traxbot.mov_lin3M_pid((byte)1,(byte)vMove_1,(byte)1,(byte)vMove_2,(byte)1,(byte)0);
  
  } else if (action==1) {
    traxbot.mov_lin3M_pid((byte)1,(byte)vMove_1,(byte)2,(byte)vMove_2,(byte)1,(byte)0);
  
  } else {
    traxbot.mov_lin3M_pid((byte)2,(byte)vMove_1,(byte)1,(byte)vMove_2,(byte)1,(byte)0);
  }
  
  
}

void stopMotorsCb(const std_msgs::Empty& msg3) {
  
    traxbot.stop_motors();
}

void encodersReadCb(const std_msgs::Empty& msg4){
  
  //delay(50);
  int enco1 = abs(traxbot.read_enc1()+32768);
  
  encoderRead1.data = enco1;
  pub_encoderRead1.publish(&encoderRead1);
  
  //delay(50);
  int enco2 = abs(traxbot.read_enc2()+32768);
  
  encoderRead2.data = enco2;
  pub_encoderRead2.publish(&encoderRead2);
  
}
  
void encodersResetCb(const std_msgs::Empty& msg5){     // This function resets the encoder values to 0
 
  delay(50);
  traxbot.set_enc_value((byte)1,(word)32768);
  delay(50);
  traxbot.set_enc_value((byte)2,(word)32768);
  //delay(50);

}


/***********************  Define subscriber topics  ***********************/

// INFO
ros::Subscriber<std_msgs::Empty> sub_info("/traxbotInfo", infoCb );

// MOTORS
ros::Subscriber<std_msgs::Empty> sub_encodersRead( "/encodersRead", encodersReadCb);
ros::Subscriber<std_msgs::Empty> sub_encodersReset( "/encodersReset", encodersResetCb);
ros::Subscriber<geometry_msgs::Point32> sub_moveMotors("/moveMotors", moveMotorsCb );
ros::Subscriber<std_msgs::Empty> sub_stopMotors("/stopMotors", stopMotorsCb );



/******************************  Setup  ************************************/
void setup()
{
  std_msgs::Empty enc_reset;

  nh.initNode();

  /* Advertise */

  // INFO  
  nh.advertise(pub_driverFirmware);
  nh.advertise(pub_batteryPower);
  nh.advertise(pub_driverTemperature);
  nh.advertise(pub_robotID);

  // MOTORS  
  nh.advertise(pub_encoderRead1);
  nh.advertise(pub_encoderRead2);

  // SONARS
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);

  
  /* Subscribe */
  
  // INFO
  nh.subscribe(sub_info);
  
  // MOTORS  
  nh.subscribe(sub_moveMotors);
  nh.subscribe(sub_stopMotors);
  nh.subscribe(sub_encodersReset);
  nh.subscribe(sub_encodersRead);


  /* Give 5v to power sonars, digital pin 13 */
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  /* Give 5v to digital pin 12 */
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  
  delay(250);	// Wait 250ms before reading sonars


  /* Config motor driver Omni3MD */
  traxbot.set_prescaler((byte)1,(byte)0);          //resolution of encoders define as 1
  delay(50);
  traxbot.set_prescaler((byte)2,(byte)0);
  delay(50);
  traxbot.set_prescaler((byte)3,(byte)0);
  delay(50);
  traxbot.set_enc_value((byte)3,(word)1000);
  delay(50);

  /* Safe */
  traxbot.stop_motors();
  encodersResetCb(enc_reset);

  /* Set PID */
  traxbot.set_PID((word)Kp,(word)Ki,(word)Kd);
  delay(50);


  /* Defines particular parameters to each robot */
  switch (ROBOT)
  {
  case 1:
    vR_1e = 3;
    vR_2e = 0;
    vR_1d = 0;
    vR_2d = 0;
    vM_1  = 0;
    vM_2  = 0;
    break;
  case 2:
    vR_1e = 0; 
    vR_2e = 0;
    vR_1d = 0;
    vR_2d = 0;
    vM_1  = 0;
    vM_2  = 2;
    break;
  case 3:
    vR_1e = 1;
    vR_2e = 0;
    vR_1d = 0;
    vR_2d = 4;
    vM_1  = 0;
    vM_2  = 4;
    break;
  default:
    break;
  }

  /* I2C connection */
  traxbot.i2c_connect(OMNI3MD_ADDRESS); //set i2c connection  

}


void loop()
{
  readSonars ();
  nh.spinOnce();
}








