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
* Author: Gonçalo Cabrita on 28/05/2012
*********************************************************************/

#include <ros/ros.h>
#include <can_msgs/CANFrame.h>

#define PC 0
#define JOYSTICK 10

//! Joystick functions
typedef enum _JoystickFunction {
    
    TurnOff = 0,
    TurnOn = 1,
    DataFromJoystick = 4
    
} EncoderFunction;
    
/*!
 *
 * \fn void turnJoystickOn()
 * \brief This functions turns the Joystick on the RobChair ON - Running Mode.
 *
 */
void turnJoystickOn(){ sendCANFrame(JOYSTICK, TurnOn, NULL, 0); }
/*!
 *
 * \fn void turnJoystickOff()
 * \brief This functions turns the Joystick on the RobChair OFF - Running Mode.
 *
 */
void turnJoystickOff(){ sendCANFrame(JOYSTICK, TurnOff, NULL, 0); }
    
/*!
 *
 * \fn void sendCANFrame(RobChairDS destination, char function, char * data, int data_count)
 * \brief This functions sends CAN frames to the RobChair CAN bus.
 *
 */
void sendCANFrame(RobChairDS destination, char function, char * data, int data_count);
/*!
 *
 * \fn void receivedCANFrame(const can_msgs::CANFrame::ConstPtr& frame_msg)
 * \brief Callback for incoming CAN frames.
 *
 */
void receivedCANFrame(const can_msgs::CANFrame::ConstPtr& frame_msg);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robchair_joystick_node");
    
	ROS_INFO("RobChair Joystick for ROS");
    
    ros::NodeHandle n;
	ros::NodeHandle n("~");
    
    ros::Rate r(10.0);
	while(n.ok())
	{
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}

// EOF
