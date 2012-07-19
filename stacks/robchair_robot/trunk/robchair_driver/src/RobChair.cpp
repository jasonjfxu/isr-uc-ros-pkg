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

#include "RobChair.h"

RobChair::RobChair()
{
    
}

RobChair::~RobChair()
{
    turnTriggerOff();
    
    turnPDrivesOff();
    turnEncodersOff();
}

void RobChair::initializeRobChair(ros::Publisher * can_pub, double kp, double ki)
{
    can_pub_ = can_pub;
    
    got_left_encoder_data_ = false;
    got_right_encoder_data_ = false;
    
    n_ = 0;
    
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    
    char data = 1;
    // Set the PDrives to receive commands from the Encoders
    sendCANFrame(RobChair::BothPDrives, RobChair::SetPDriveControlMode, &data, 1);
    // Set the Encoders to send commands to the PDrives
    sendCANFrame(RobChair::BothEncoders, RobChair::SetEncoderControlMode, &data, 1);
    
    if(kp>0.0 && ki>0.0)
    {
        setPIControlValues(RobChair::BothEncoders, kp, ki);
    }
    
    turnEncodersOn();
    turnPDrivesOn();
    
    turnTriggerOn();
}

void RobChair::setPIControlValues(RobChairDS destination, double kp, double ki)
{
    // Convert the gains to integers according to the documentation
    long kp_int = kp*1000;
    long ki_int = ki*1000;
    
    char data[8];
    
    memset(data, kp_int, 4);
    memset(data+4, ki_int, 4);
    
    sendCANFrame(destination, RobChair::SetPIControlValues, data, 8);
}

void RobChair::setVelocities(double linear_velocity, double angular_velocity)
{
    double delta_v = (ROBCHAIR_AXLE_LENGTH * angular_velocity)/2.0;
    
    // Left wheel velocity
    double left_velocity = ((linear_velocity - delta_v)/ROBCHAIR_WHEEL_RADIUS) * ROBCHAIR_K;
    // Right wheel velocity
	double right_velocity = ((linear_velocity + delta_v)/ROBCHAIR_WHEEL_RADIUS) * ROBCHAIR_K;
    
    char data[2];
    
    data[0] = static_cast<int>(left_velocity) & 0x0FF;
    data[1] = (static_cast<int>(left_velocity) >> 8) & 0x0FF;
    // Reference velocity must be sent at N=3
    sendCANFrame(RobChair::LeftEncoder, RobChair::SetVelocityValue, data, 2, 3);
    
    data[0] = static_cast<int>(right_velocity) & 0x0FF;
    data[1] = (static_cast<int>(right_velocity) >> 8) & 0x0FF;
    // Reference velocity must be sent at N=3
    sendCANFrame(RobChair::RightEncoder, RobChair::SetVelocityValue, data, 2, 3);
}

void RobChair::decodeEncoderData(RobChairDS encoder, const can_msgs::CANFrame::ConstPtr& msg)
{
    double temp_wheel_position;
	double temp_wheel_velocity;
    
    temp_wheel_position = (static_cast<int>(msg->data[3]) << 24) + (static_cast<int>(msg->data[2]) << 16) + (static_cast<int>(msg->data[1]) << 8) + static_cast<int>(msg->data[0]);
	temp_wheel_velocity = (static_cast<int>(msg->data[5]) << 8) + static_cast<int>(msg->data[4]);
    
    temp_wheel_position = static_cast<double>(temp_wheel_position)*(ROBCHAIR_CONTROL_PERIOD/ROBCHAIR_K);
	temp_wheel_velocity = static_cast<double>(temp_wheel_velocity)*(1/ROBCHAIR_K);
                             
    if(encoder == RobChair::LeftEncoder)
    {
        last_left_wheel_position_ = left_wheel_position_;
        left_wheel_position_ = temp_wheel_position * ROBCHAIR_WHEEL_RADIUS;
        left_wheel_velocity_ = temp_wheel_velocity * ROBCHAIR_WHEEL_RADIUS;
        got_left_encoder_data_ = true;
    }
    else if(encoder == RobChair::RightEncoder)
    {
        last_right_wheel_position_ = right_wheel_position_;
        right_wheel_position_ = temp_wheel_position * ROBCHAIR_WHEEL_RADIUS;
        right_wheel_velocity_ = temp_wheel_velocity * ROBCHAIR_WHEEL_RADIUS;
        got_right_encoder_data_ = true;
    }
    
    // If we got both encoders data lets process it!
    if(got_left_encoder_data_ && got_left_encoder_data_)
    {
        double delta_phi_left = left_wheel_position_ - last_left_wheel_position_;
        double delta_phi_right = right_wheel_position_ - last_right_wheel_position_;
        
        double delta_s = (delta_phi_right + delta_phi_left)/2;
        double delta_theta = (delta_phi_right - delta_phi_left)/ROBCHAIR_AXLE_LENGTH;

        double half_theta = limitAngle(yaw_ + delta_theta/2);
        
        x_ += delta_s * cos(half_theta);
        y_ += delta_s * sin(half_theta);
        yaw_ = limitAngle(yaw_ + delta_theta);
        
        linear_velocity_ = (right_wheel_velocity_ + left_wheel_velocity_)/2;
        angular_velocity_ = (right_wheel_velocity_ - left_wheel_velocity_)/ROBCHAIR_AXLE_LENGTH;
        
        got_left_encoder_data_ = false;
        got_right_encoder_data_ = false;
    }
}

void RobChair::sendCANFrame(RobChairDS destination, char function, char * data, int data_count, char n)
{
    can_msgs::CANFrame frame;
    
    frame.stamp = ros::Time::now();
    
    frame.id = destination << 7 + RobChair::PC << 4 + function;
    frame.data.resize(data_count);
    for(int i=0 ; i<data_count ; i++)
    {
        frame.data[i] = data[i];
    }
    
    // If a valid trigger time slot is provided...
    if(n>0 && n<=5)
    {
        // Wait for it!
        while(n_!=n)
        {
            ros::Duration(ROBCHAIR_CONTROL_PERIOD/5.0/10.0).sleep();
        }
    }
    
    can_pub_->publish(frame);
}

void RobChair::receivedCANFrame(const can_msgs::CANFrame::ConstPtr& msg)
{
    ROS_INFO("RobChair - %s - Got a message!", __FUNCTION__);
    
    // First lets see what we got here...
    char function = msg->id & ROBCHAIR_FUNCTION_MASK;
    char source = msg->id & ROBCHAIR_SOURCE_MASK;
    char destination = msg->id & ROBCHAIR_DESTINATION_MASK;
    
    ROS_INFO("RobChair - %s - D:%d S:%d F:%d", __FUNCTION__, destination, source, function);

    // If the message is not meant for us drop it
    if(destination != RobChair::PC) return;
    
    // Now lets check who sent it...
    switch(source)
    {
        case RobChair::RightEncoder:
            if(function == RobChair::DataFromEncoder)
            {
                decodeEncoderData(RobChair::RightEncoder, msg);
            }
            break;
        
        case RobChair::LeftEncoder:
            if(function == RobChair::DataFromEncoder)
            {
                decodeEncoderData(RobChair::LeftEncoder, msg);
            }
            break;
            
        case RobChair::SyncMCU:
            if(function == RobChair::SyncronizeAllNodes)
            {
                n_ = msg->data[0];
            }
            break;
            
        default:
            ROS_ERROR("RobChair - %s - Got CAN frame from an unknown source: %d", __FUNCTION__, source);
            break;
    }
}

double RobChair::limitAngle(double angle)
{
    double correct_angle;
    
    if(angle > M_PI)
    {
        correct_angle = angle - 2 * M_PI;
    }
    else if(angle < - M_PI)
    {
        correct_angle = angle + 2 * M_PI;
    }
    else
    {
        correct_angle = angle;
    }
    
    return correct_angle;
}


// EOF
