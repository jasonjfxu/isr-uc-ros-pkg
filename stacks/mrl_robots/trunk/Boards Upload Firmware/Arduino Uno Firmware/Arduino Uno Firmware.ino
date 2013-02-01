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
* Modified by:   Andre Araujo & David Portugal on 04/01/2013
* Version: Traxbot_Stingbot_DriverROS_v1
*********************************************************************/

// Arduino libraries
#include <EEPROM.h>
#include <stdlib.h>
#include <Wire.h>
#include <string.h>
#include <math.h>

// Traxbot robot & Omni3MD lib
#include "Robot.h"

// RobotSerial Communication lib
#include "RobotSerialComm.h"

Omni3MD omni;
Robot robot;

// Arguments for the reply
unsigned int reply_arg[5];

// RobotSerial Communication
RobotSerialComm port;

//Streaming without interrupt:
boolean stream=false;


//Variables for motion control
double lin_speed_si=0.0;    //Linear speed in m/s
double ang_speed_si=0.0;    //Rotational speed in rad/s


// ******* Helper functions *******

void sendEncodersReads(){  
  
    reply_arg[0] = omni.read_enc3();
    reply_arg[1] = omni.read_enc1();
    port.reply(OMNI_READ_ENCODERS, reply_arg, 2);
}

void sendSonarsReads(){
  
    reply_arg[0] = robot.getRange(FRONT_SONAR);
    reply_arg[1] = robot.getRange(LEFT_SONAR);
    reply_arg[2] = robot.getRange(RIGHT_SONAR);
    port.reply(READ_SONARS, reply_arg, 3);
}

void sendEncodersSonarsReads(){
  
    reply_arg[0] = omni.read_enc3();
    reply_arg[1] = omni.read_enc1();
    reply_arg[2] = robot.getRange(FRONT_SONAR);
    reply_arg[3] = robot.getRange(LEFT_SONAR);
    reply_arg[4] = robot.getRange(RIGHT_SONAR);
    port.reply(READ_ENCODERS_SONARS, reply_arg, 5);
}

void sendRobotInfo(){   
  
    double value = robot.EEPROMReadDouble(0); //read Address "0" robot id  (expected to be written previously)
    reply_arg[0] = round( omni.read_temperature() * 100 );
    reply_arg[1] = round( omni.read_firmware() * 100 );
    reply_arg[2] = round( omni.read_battery() * 100 );
    reply_arg[3] = ROBOT_FIRMWARE_VERSION;
    reply_arg[4] = (int)value;                // Robot ID
    port.reply(ROBOT_INFO, reply_arg, 5);
}


// ******* Setup *******
void setup(){
  
     
  // Serial port stuff
  Serial.begin(BAUD_RATE);  // defined in "Robot.h"
    
  // I2C connection
  omni.i2c_connect(OMNI3MD_ADDRESS); //set i2c connection
  delay(10);                           // pause 10 milliseconds

  omni.stop_motors();                 // stops all motors
  delay(10);  

  omni.set_i2c_timeout(0); // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
  delay(5);                 // 5ms pause required for Omni3MD eeprom writing

  // Encoders Reset
  omni.set_enc_value(1,0);   // presets the encoder value [byte encoder, word encValue]
  omni.set_enc_value(3,0);   // presets the encoder value [byte encoder, word encValue]
   
  //set_prescaler(byte encoder, byte value)  value: 0 - 1 pulse; 1 - 10 pulses; 2 - 100 pulses; 3 - 1000 pulses; 4 - 10000 pulses  (requires 10ms) 
  omni.set_prescaler(1, 0);  //sets the prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
  delay(10);                 // 10ms pause required for Omni3MD eeprom writing
  omni.set_prescaler(3, 0);
  delay(10);
 
 
  double robot_id = robot.EEPROMReadDouble(0);
  word ramp_time;
  double axis_radius,whell_radius; 

  if (robot_id <=3) {        // To TraxBot #1,#2,#3
    ramp_time = 2500;
    axis_radius = 87;
    whell_radius = 27;
  } else {                   // To StingBot #4,#5
    ramp_time = 1500;
    axis_radius = 110;
    whell_radius = 19;
  }
  
  omni.set_PID(Kp,Ki,Kd); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
  delay(15);                 // 15ms pause required for Omni3MD eeprom writing
  
  omni.set_ramp(ramp_time,0);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word Kl] 
  delay(10);                 // 10ms pause required for Omni3MD eeprom writing  

  omni.set_differential(axis_radius,whell_radius,gearbox_factor,encoder_cpr);
  delay(20);
   
  // Give 5v to power sonars, digital pin 13
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  // Give 5v to digital pin 12 
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  
}

// ******* Main loop *******
void loop(){
  
    unsigned int arg[5];
    int action = port.getMsg(arg);  
    
    if(action==0 && stream==true){
      action=ACTION_START_STREAM;
    }    

    // If we got an action...Process it:
    switch(action){
          
            case OMNI_CALIBRATION:                 // "@1e", no reply
                omni.calibrate(1,0,0);
                delay(95);
                break;
            
            case OMNI_SET_PID:                    //@2,"KP","KI","KD"e, no reply
                omni.set_PID(arg[0], arg[1], arg[2]);
                break;
                
            case OMNI_SET_PRESCALER:              //@3,"enc","value"e, no reply
                omni.set_prescaler(arg[0], arg[1]);
                break;

            case OMNI_SET_ENC_VALUE:            //@4,"enc","enc_value"e, no reply
                omni.set_enc_value(arg[0], arg[1]);
                break;

            case ROBOT_INFO:                    //@5e, reply: @5,"temp","firm","bat","r_firm","r_id"e
                sendRobotInfo();
                break;

            case OMNI_READ_ENCODERS:          //@6e,  reply: @6,"enc1(R)","enc2(L)"e
                sendEncodersReads();
                break;

            case READ_SONARS:                //@7e, reply: @7,"son1(F)","son2(L)","son3(R)"e
                sendSonarsReads();
                break;    

            case READ_ENCODERS_SONARS:       //@8e, reply: @8,"enc1(R)","enc2(L)","son1(F)","son2(L)","son3(R)"e
                sendEncodersSonarsReads();
                break;    
                
            case LINEAR_MOVE_PID:            //@9,"speed1","speed3"e, no reply
                omni.mov_lin3m_pid(arg[0], 0, arg[1]);              
                break;

            case LINEAR_MOVE_NOPID:          //@10,"speed1","speed2"e, no reply
                omni.mov_lin3m_nopid(arg[0], 0, arg[1]);
                break;

            case MOVE_DIFFERENTIAL_SI:          //@11,"vel_linear","vel_angular"e, no reply
                lin_speed_si= ((double)arg[0]/1000); 
                ang_speed_si= ((double)arg[1]/1000);
                omni.mov_dif_si(lin_speed_si, ang_speed_si);
                break;

            case MOVE_POSITIONAL:              //@12,"motor_nr","speed","encoder_Position"e, no reply
                omni.mov_pos(arg[0], arg[1], arg[2], 1);  // move motor1 at speed1 until encoder count reaches the defined position and then stop with holding torque
                delay(1);                          // wait 1ms for Omni3MD to process information
                break;

            case STOP_MOTORS:                //@13e, no reply
                omni.stop_motors();
                break;
                
            case ENCODERS_RESET:             //@14e, no reply
                robot.encodersReset();
                break;                
                              
            case ACTION_GET_DEBUG:           //@15e, reply (to the console): @13,"0/1"e
                reply_arg[0] = port.getDebug();
                port.reply(ACTION_GET_DEBUG, reply_arg, 1);
                break;

            case ACTION_SET_DEBUG:           //@16,"0/1"e, no reply
                port.setDebug(arg[0]);  
                break;

            case ACTION_GET_STREAM:           //@17e, reply @15,"0/1"e
                reply_arg[0] = stream;
                port.reply(ACTION_GET_STREAM, reply_arg, 1);
                break;
                
            case ACTION_START_STREAM:      // "@18e,  reply: @6,"enc1(R)","enc2(L)"e (repeatedly)
                stream = true;
                sendEncodersReads();
                //delay(65);                //encoders read update (+- 15Hz)
                break;
                
            case ACTION_STOP_STREAM:        // "@19e,  no reply 
                stream = false;                
                break;
                
            default:
                break;

        
   } // switch
    
} // loop()

// EOF









