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
* Author: Gonçalo Cabrita on 06/06/2013
*********************************************************************/

#include <ros/ros.h>
#include "Squirtle.h"

cereal::CerealPort Squirtle::serial_port;

Squirtle::Squirtle()
{
	
}

Squirtle::~Squirtle()
{
	
}

bool Squirtle::openPort(char * port, int baudrate)
{
    try{ serial_port.open(port, baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

bool Squirtle::initialize()
{
    char msg_to_send[64];

    int num_bytes = sprintf(msg_to_send, ";AHR;");
    serial_port.write(msg_to_send, num_bytes);

    if(!serial_port.startReadBetweenStream(boost::bind(&Squirtle::streamCallback, this, _1), ':', ';'))
    {
        return false;
    }
    return true;
}

bool Squirtle::startStreaming(int motors, int fluorometer, int pm)
{
    char msg_to_send[64];

    int num_bytes = sprintf(msg_to_send, "ASC M %d F %d P %d;", motors, fluorometer, pm);
    try{ serial_port.write(msg_to_send,num_bytes); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

void Squirtle::streamCallback(std::string * msg)
{
    int scan_res;
    char h1[10];

    scan_res = sscanf(msg->c_str(), ":%s %*s", h1);

    if(scan_res == 1)                                   // must scan 1 entry
    {
        if(!strncmp(h1,"CMD",3))                        // header
        {
            parseMotorData(msg->c_str());               // Parse driver msgs
            return;
        }
        else if(!strncmp(h1,"FCV",3))                   // header
        {
            parseChemicalSensorData(msg->c_str());      // Parse cyclops msgs
            return;
        }
        else if(!strncmp(h1,"BLV",3))                   // header
        {
            parsePowerData(msg->c_str());               // Parse pm msgs
            return;
        }
        else if(!strncmp(h1,"LMT",3))                   // header
        {
            parseLM35Data(msg->c_str());                // Parse lm35 msgs
            return;
        }
        else if(!strncmp(h1,"START",5))                 // header
        {
            arduino_ready_ = true;
            return;
        }
        else
        {
            // Error!
            return;
        }
    }

}

bool Squirtle::parseMotorData(const char * msg)
{
    int d1=0, d2=0, d3=0, d4=0, scan_res;
    char h1[3],h2[1],h3[1],h4[2],h5[2];

    //scan message
    scan_res=sscanf(msg, ":%s %c %d %c %d %s %d %s %d;%*s",h1, h2, &d1, h3, &d2, h4, &d3,  h5, &d4);

    if(scan_res == 9 && !strcmp(h1,"CMD") && h2[0]=='L' && h3[0]=='R' && !strcmp(h4,"PL") && !strcmp(h5,"PR"))
    {
        return false;
    }

    left_motor_current_ = d1;
    right_motor_current_ = d2;

    return true;
}

bool Squirtle::parseChemicalSensorData(const char * msg)
{
    int d1=0, d2=0, scan_res;
    char h1[4],h2[1],h3[1];

    scan_res=sscanf(msg, ":%s %c %d %c %d;%*s",h1, h2, &d1, h3, &d2);

    if(scan_res == 5 && !strcmp(h1,"FCV") && h2[0]=='V' && h3[0]=='G')						// must scan 5 entries
    {
        return false;
    }

    chemical_sensor_reading_ = d1;

    return true;
}

bool Squirtle::parsePowerData(const char * msg)
{
    int d1=0, d2=0, d3=0, scan_res;
    char h1[5],h2[1],h4[1],h5[1];

    scan_res=sscanf(msg, ":%s %c %d %c %d %c %d;%*s",h1,h2,&d1,h4,&d2,h5,&d3);

    if(scan_res == 7 && !strcmp(h1,"BLV") && h2[0] == 'B' && h4[0]=='P' && h5[0]=='C')
    {
        return false;
    }

    battery_voltage_ = (double)( d1/1.878 )/1000.0;                // in volts
    battery_current_ = 0.0;
    battery_temperature_ = 0.0;
    solar_panel_voltage_ = (double)( d2/0.76436 )/1000.0;          // in volts
    solar_panel_current_ = (double)( (d3 - 510)/-0.01506 )/1000.0; // in amps

    return true;
}

bool Squirtle::parseLM35Data(const char * msg)
{
    // Todo...
    return true;
}

void Squirtle::drive(int left_motor, int right_motor, int linear_actuator)
{
    if(left_motor > SQUIRTLE_MAX_MOTOR) left_motor = SQUIRTLE_MAX_MOTOR;
    if(right_motor > SQUIRTLE_MAX_MOTOR) right_motor = SQUIRTLE_MAX_MOTOR;
    if(left_motor < SQUIRTLE_MIN_MOTOR) left_motor = SQUIRTLE_MIN_MOTOR;
    if(right_motor < SQUIRTLE_MIN_MOTOR) right_motor = SQUIRTLE_MIN_MOTOR;
    if(linear_actuator > SQUIRTLE_MAX_LINEAR_ACTUATOR) linear_actuator = SQUIRTLE_MAX_LINEAR_ACTUATOR;
    if(linear_actuator < SQUIRTLE_MIN_LINEAR_ACTUATOR) linear_actuator = SQUIRTLE_MIN_LINEAR_ACTUATOR;

    int num_bytes;
    char msg_to_send[64];

    // Message to motors
    num_bytes = sprintf(msg_to_send, "MDC L %d R %d;", left_motor, right_motor);
    serial_port.write(msg_to_send, num_bytes);

    // Message to linear actuator
    num_bytes = sprintf(msg_to_send, "MDO O %d;", linear_actuator);
    serial_port.write(msg_to_send, num_bytes);
}

// EOF

