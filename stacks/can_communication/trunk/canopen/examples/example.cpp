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
 * Author: Gonçalo Cabrita on 17/04/2012
 *********************************************************************/

#include <ros/ros.h>
#include <canopen/CANopenMotorDriver.h>

#define AXIS_ID 1

//#define ENCODER_LINES_PER_REVOLUTION 500
//#define Tr 9213.0/9.0
//#define T 0.001

// Internal units conversion    
double no_usteps = 256;     // Number of micro-steps p/ step
double no_steps = 200;      // Number of steps p/ revolution
double Tr = 2*M_PI*0.05;    // Relation between motor and load
//double Tr = 2*M_PI*5;    // Relation between motor and load
double T = 0.0008;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "canopen_motor_driver_example_node");
  	
  	ROS_INFO("CANopenMotorDriver Example Node");
    
    ros::NodeHandle nh;
    
    boost::thread t = boost::thread::thread(boost::bind(&ros::spin));
    
    canopen::CANopenMotorDriver motor(AXIS_ID, &nh);
    
    //motor.setupDCmotorWithEncoder(ENCODER_LINES_PER_REVOLUTION, Tr, T);
    motor.setupStepperMotor(no_steps, no_usteps, Tr, T);
    
    motor.startNode();
    
    ros::Duration(1.0).sleep();
    
    motor.readyToSwitchOn();
    
    ros::Duration(1.0).sleep();
    
    motor.switchOn();
    
    ros::Duration(2.0).sleep();
    
    motor.enableOperation();
    
    ros::Duration(1.0).sleep();
    
    motor.setupHomingMode(canopen::method_18, 20.0, 10.0, 10.0);
    //motor.setupPositionTrapezoidalProfileMode(10.0, 20.0, 10.0);
    //motor.setupVelocityProfileMode(0.05, 0.05);
    
    ros::Duration(1.0).sleep();
    
    motor.run();
    
    ros::Rate r(1);
  	while(ros::ok())
	{
        r.sleep();
	}
    
    t.join();

    return(0);
}

//EOF
