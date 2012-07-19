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

#include <canopen/CANopenMotorDriver.h>

//! Macro for throwing an exception with a message, passing args
/*#define CANOPEN_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in canopen::CANopenMotorDriver::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}*/

canopen::CANopenMotorDriver::CANopenMotorDriver(unsigned char axis_id, ros::NodeHandle * nh) //: nh_()
{
	id_ = axis_id;
    
    can_pub_ = nh->advertise<can_msgs::CANFrame>("/can_bus_tx", 10);
    can_sub_ = nh->subscribe<can_msgs::CANFrame>("/can_bus_rx", 10, &canopen::CANopenMotorDriver::receivedCANframe, this);
    
    ros::Duration(0.5).sleep();
}

canopen::CANopenMotorDriver::~CANopenMotorDriver()
{
    
}

void canopen::CANopenMotorDriver::receivedCANframe(const can_msgs::CANFrame::ConstPtr& msg)
{
    ROS_INFO("Got a CANFrame!!!");
    
    // If this message is not intended for this axis return
    if(msg->cob_id & ID_MASK != id_) return;
}



/***************************************** SCALING FACTORS STUFF *****************************************/

void canopen::CANopenMotorDriver::setupDCmotorWithEncoder(unsigned int num_of_encoder_lines, double Tr, double T)
{    
    // Time units
    T_ = T;
    
    // Position units
    load_to_motor_position_ = 2*M_PI/(4*num_of_encoder_lines*Tr);
    
    // Speed units
    load_to_motor_speed_ = 2*M_PI/(4*num_of_encoder_lines*Tr*T);
    
    // Acceleration units
    load_to_motor_acceleration_ = 2*M_PI/(4*num_of_encoder_lines*Tr*T*T);
    
    // Jerk units
    load_to_motor_jerk_ = 2*M_PI/(4*num_of_encoder_lines*Tr*T*T*T);
}

void canopen::CANopenMotorDriver::setupStepperMotor(unsigned int num_of_steps, unsigned int num_of_usteps, double Tr, double T)
{
    // Time units
    T_ = T;
    
    // Position units
    load_to_motor_position_ = 2*M_PI/(num_of_steps*num_of_usteps*Tr);
    
    // Speed units
    load_to_motor_speed_ = 2*M_PI/(num_of_steps*num_of_usteps*Tr*T);
    
    // Acceleration units
    load_to_motor_acceleration_ = 2*M_PI/(num_of_steps*num_of_usteps*Tr*T*T);
    
    // Jerk units
    load_to_motor_jerk_ = 2*M_PI/(num_of_steps*num_of_usteps*Tr*T*T*T);
}



/***************************************** GENERAL STUFF *****************************************/

void canopen::CANopenMotorDriver::startNode()
{
    ROS_INFO("CANopenMotorDriver - %s", __FUNCTION__);
    
    can_msgs::CANFrame frame;
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_NMT_SERVICE;
    frame.data.resize(2);
    frame.data[0] = 0x01;
    frame.data[1] = id_;
    can_pub_.publish(frame);
}

void canopen::CANopenMotorDriver::readyToSwitchOn()
{
    ROS_INFO("CANopenMotorDriver - %s", __FUNCTION__);
    
    can_msgs::CANFrame frame;
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_PDO1+id_;
    frame.data.resize(2);
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    can_pub_.publish(frame);
}

void canopen::CANopenMotorDriver::switchOn()
{
    ROS_INFO("CANopenMotorDriver - %s", __FUNCTION__);
    
    can_msgs::CANFrame frame;
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_PDO1+id_;
    frame.data.resize(2);
    frame.data[0] = 0x07;
    frame.data[1] = 0x00;
    can_pub_.publish(frame);
}

void canopen::CANopenMotorDriver::enableOperation()
{
    ROS_INFO("CANopenMotorDriver - %s", __FUNCTION__);
    
    can_msgs::CANFrame frame;
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_PDO1+id_;
    frame.data.resize(2);
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    can_pub_.publish(frame);
}

void canopen::CANopenMotorDriver::run()
{
    ROS_INFO("CANopenMotorDriver - %s", __FUNCTION__);
    
    can_msgs::CANFrame frame;
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_PDO1+id_;
    frame.data.resize(2);
    frame.data[0] = 0x1F;
    frame.data[1] = 0x00;
    can_pub_.publish(frame);
}



/***************************************** HOMING MODE *****************************************/

void canopen::CANopenMotorDriver::setupHomingMode(canopen::HomingMethod method, double fast_speed, double slow_speed, double acceleration, double offset)
{
    can_msgs::CANFrame frame;
    
    int homing_speed_iu = (int)(fast_speed/load_to_motor_speed_*65535);
    
    ROS_INFO("CANopenMotorDriver - %s - Homing Speed: %d", __FUNCTION__, homing_speed_iu);
    
    // Homing speed during search for switch
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x99;
    frame.data[2] = 0x60;
    frame.data[3] = 0x01;
    frame.data[4] = homing_speed_iu;
    frame.data[5] = homing_speed_iu>>8;
    frame.data[6] = homing_speed_iu>>16;
    frame.data[7] = homing_speed_iu>>24;
    can_pub_.publish(frame);
    
    int homing_slow_speed_iu = (int)(slow_speed/load_to_motor_speed_*65535);
    
    ROS_INFO("CANopenMotorDriver - %s - Homing Slow Speed: %d", __FUNCTION__, homing_slow_speed_iu);
    
    // Homing speed during search for index
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x99;
    frame.data[2] = 0x60;
    frame.data[3] = 0x02;
    frame.data[4] = homing_slow_speed_iu;
    frame.data[5] = homing_slow_speed_iu>>8;
    frame.data[6] = homing_slow_speed_iu>>16;
    frame.data[7] = homing_slow_speed_iu>>24;
    can_pub_.publish(frame);
    
    int homing_acceleration_iu = (int)(acceleration/load_to_motor_acceleration_*65535);
    
    ROS_INFO("CANopenMotorDriver - %s - Homing Acceleration: %d", __FUNCTION__, homing_acceleration_iu);
    
    // Homing acceleration
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x9A;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = homing_acceleration_iu;
    frame.data[5] = homing_acceleration_iu>>8;
    frame.data[6] = homing_acceleration_iu>>16;
    frame.data[7] = homing_acceleration_iu>>24;
    can_pub_.publish(frame);
    
    // Homing method
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_1BYTE;
    frame.data[1] = 0x98;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = method;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    can_pub_.publish(frame);
    
    // Mode of operation - homing mode
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_1BYTE;
    frame.data[1] = 0x60;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x06;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    can_pub_.publish(frame);
    
    // TODO: Dontf forget to add the offset CAN frame!
}



/***************************************** POSITION MODE *****************************************/

void canopen::CANopenMotorDriver::setupPositionTrapezoidalProfileMode(double target, double speed, double acceleration)
{
    can_msgs::CANFrame frame;
    
    // Mode of operation - position mode
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_1BYTE;
    frame.data[1] = 0x60;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x01;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    can_pub_.publish(frame);
    
    // Motion profile type -  trapezoidal
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_2BYTE;
    frame.data[1] = 0x86;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    can_pub_.publish(frame);
    
    int speed_iu = (int)(speed/load_to_motor_speed_*65535);
    
    // Target speed
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x81;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = speed_iu;
    frame.data[5] = speed_iu>>8;
    frame.data[6] = speed_iu>>16;
    frame.data[7] = speed_iu>>24;
    can_pub_.publish(frame);
    
    int acceleration_iu = (int)(acceleration/load_to_motor_acceleration_*65535);
    
    // Profile acceleration
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x83;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = acceleration_iu;
    frame.data[5] = acceleration_iu>>8;
    frame.data[6] = acceleration_iu>>16;
    frame.data[7] = acceleration_iu>>24;
    can_pub_.publish(frame);
    
    int position_iu = target/load_to_motor_position_;
    
    // Target position
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x7A;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = position_iu;
    frame.data[5] = position_iu>>8;
    frame.data[6] = position_iu>>16;
    frame.data[7] = position_iu>>24;
    can_pub_.publish(frame);
}

void canopen::CANopenMotorDriver::setQuickStopDeceleration(double deceleration)
{
    can_msgs::CANFrame frame;
    
    int deceleration_iu = (int)(deceleration/load_to_motor_acceleration_*65535);
    
    // Quick deceleration
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x85;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = deceleration_iu;
    frame.data[5] = deceleration_iu>>8;
    frame.data[6] = deceleration_iu>>16;
    frame.data[7] = deceleration_iu>>24;
    can_pub_.publish(frame);
}



/***************************************** VELOCITY MODE *****************************************/

void canopen::CANopenMotorDriver::setupVelocityProfileMode(double target_speed, double acceleration)
{
    can_msgs::CANFrame frame;
    
    // Mode of operation - velocity mode
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_1BYTE;
    frame.data[1] = 0x60;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    can_pub_.publish(frame);
    
    int speed_iu = (int)(target_speed/load_to_motor_speed_*65535);
    
    // Target speed
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0xFF;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = speed_iu;
    frame.data[5] = speed_iu>>8;
    frame.data[6] = speed_iu>>16;
    frame.data[7] = speed_iu>>24;
    can_pub_.publish(frame);
    
    int acceleration_iu = (int)(acceleration/load_to_motor_acceleration_*65535);
    
    // Profile acceleration
    frame.stamp = ros::Time::now();
    frame.cob_id = COB_R_SDO+id_;
    frame.data.resize(8);
    frame.data[0] = WRITE_REQUEST_4BYTE;
    frame.data[1] = 0x83;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = acceleration_iu;
    frame.data[5] = acceleration_iu>>8;
    frame.data[6] = acceleration_iu>>16;
    frame.data[7] = acceleration_iu>>24;
    can_pub_.publish(frame);
}

// EOF
