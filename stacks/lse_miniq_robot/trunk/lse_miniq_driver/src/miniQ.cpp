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
* Author: Gonçalo Cabrita on 20/08/2012
*********************************************************************/

#include <ros/ros.h>
#include "miniQ.h"

cereal::CerealPort miniQ::serial_port;

miniQ::miniQ()
{
    id_ = 0;
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
}

miniQ::~miniQ()
{
    
}

bool miniQ::openPort(char * port, int baudrate)
{
    try{ serial_port.open(port, baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

bool miniQ::checkVersion()
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%de", id_, MQ_ACTION_GET_VERSION);
    
    serial_port.write(msg);
    
    std::string reply;
    try{ serial_port.readBetween(&reply, '@', 'e', 100); }
    catch(cereal::TimeoutException& e)
    {
     //   return false;
    }
    
    //ROS_INFO("Reply: %s", reply.c_str());
    
    int id, action, version;
    sscanf(reply.c_str(), "@%d,%d,%de", &id, &action, &version);
    
    if(version != 3) return false;

    id_ = id;
    
    return true;
}

void miniQ::setVelocities(double linear_velocity, double angular_velocity)
{
    linear_velocity_= linear_velocity;
    angular_velocity_ = angular_velocity;
}

void miniQ::setPWM(int left_pwm, int left_dir, int right_pwm, int right_dir)
{   
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%d,%d,%de", id_, MQ_ACTION_DRIVE_DIRECT, left_pwm, left_dir, right_pwm, right_dir);
    
    serial_port.write(msg);
}

bool miniQ::update()
{
    int linear_velocity_int = (int)(linear_velocity_*1000);
    int angular_velocity_int = (int)(angular_velocity_*1000);
    
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%d,%de", id_, MQ_ACTION_DRIVE, linear_velocity_int, angular_velocity_int, MQ_ACTION_GET_GROUP_1);
    
    //ROS_INFO("Drive: %s", msg);

    serial_port.write(msg);
    
    std::string reply;
    try{ serial_port.readBetween(&reply, '@', 'e', 50); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    
    int id, action, x, y, yaw, gas, gas_raw;
    sscanf(reply.c_str(), "@%d,%d,%d,%d,%d,%d,%de", &id, &action, &x, &y, &yaw, &gas, &gas_raw);
    
    x_ = x/1000.0;
    y_ = y/1000.0;
    yaw_ = yaw/1000.0;

    double normalized_gas = (gas_raw-5)/500.0;
    if(normalized_gas < 0.0) normalized_gas = 0.0;
    else if(normalized_gas > 1.0) normalized_gas = 1.0;
    gas_ = normalized_gas;
    gas_raw_ = gas_raw;
    
    return true;
}

bool miniQ::updateVelocities()
{
    int linear_velocity_int = (int)(linear_velocity_*1000);
    int angular_velocity_int = (int)(angular_velocity_*1000);
    
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%d,%de", id_, MQ_ACTION_DRIVE, linear_velocity_int, angular_velocity_int, 0);
    
    //ROS_INFO("Drive: %s", msg);
    
    serial_port.write(msg);
    
    return true;
}

void miniQ::setId(int id)
{
    id_ = id;
}

int miniQ::scanForId(int id)
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%de", id, MQ_ACTION_GET_VERSION);
    
    serial_port.write(msg);
    
    std::string reply;
    try{ serial_port.readBetween(&reply, '@', 'e', 100); }
    catch(cereal::TimeoutException& e)
    {
        return 0;
    }
    
    int robot_id, action, version, calibrated;
    sscanf(reply.c_str(), "@%d,%d,%d,%de", &robot_id, &action, &version, &calibrated);
    
    if(version > 0) return id;
    return 0;
}

bool miniQ::setMode(int mode)
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%de", id_, MQ_ACTION_SET_MODE, mode);
    
    serial_port.write(msg);

    return true;
}

bool miniQ::setBatteryType(int battery_type)
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%de", id_, MQ_ACTION_SET_BATTERY_TYPE, battery_type);
    
    serial_port.write(msg);

    return true;
}

bool miniQ::setPIDGains(int kp, int ki, int kd, miniQPID pid)
{
    char msg[MSG_LENGTH];
    
    int side, stage;
    if(pid == LeftStartingPID)
    {
	side = 0;
	stage = 0;
    }
    else if(pid == LeftRunningPID)
    {
	side = 0;
	stage = 1;
    }
    else if(pid == RightStartingPID)
    {
	side = 1;
	stage = 0;
    }
    else if(pid == RightRunningPID)
    {
	side = 1;
	stage = 1;
    }
    
    sprintf(msg, "@%d,%d,%d,%d,%d,%d,%de", id_, MQ_ACTION_SET_PID_GAINS, side, stage, kp, ki, kd);
    
    serial_port.write(msg);

    return true;	
}

bool miniQ::setOdometryCallibration(double odometry_d, double odometry_yaw)
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%de", id_, MQ_ACTION_SET_ODOMETRY_CALIBRATION, (int)(odometry_d*1000), (int)(odometry_yaw*1000));
    
    serial_port.write(msg);

    return true;
}

bool miniQ::setGasSensorCallibration(double a, double b)
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%de", id_, MQ_ACTION_SET_GAS_CALIBRATION, (int)(a*1000), (int)(b*1000));
    
    serial_port.write(msg);

    return true;
}

bool miniQ::updateWheelVelocities()
{
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%de",  id_, MQ_ACTION_GET_WHEEL_VELOCITIES);
    
    serial_port.write(msg);
    
    std::string reply;
    try{ serial_port.readBetween(&reply, '@', 'e', 50); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    
    int id, action, l, r;
    sscanf(reply.c_str(), "@%d,%d,%d,%de", &id, &action, &l, &r);
    
    left_wheel_velocity_ = l/1000.0;
    right_wheel_velocity_ = r/1000.0;
    
    return true;
}

// EOF
