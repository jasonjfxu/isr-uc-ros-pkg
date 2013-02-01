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
* Originally by: Gonçalo Cabrita on 22/09/2011
* Modified by:   Andre Araujo & David Portugal on 04/01/2013
*********************************************************************/

#include "Arduino.h"
#include "RobotSerialComm.h"

int ACTION_PARAM_COUNT[] = {0, 3, 2, 2, 0, 0, 0, 0, 2, 2, 2, 3, 0, 0, 0, 1, 0, 0, 0}; //parse number of arguments from serial port (ROS in this case)
                         // 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19

RobotSerialComm::RobotSerialComm()
{
    serial_port_status = AWATING_DATA;
    debug = 0;  // By default debug is OFF
}
    
int RobotSerialComm::getMsg(unsigned int * argv)
{
    // If data is available...
    if(Serial.available())
    {
        // Read a byte from the serial port
        byte newByte = Serial.read();
        
        // Got a start byte!!!
        if(newByte == START)
        {
            // If we are waiting for data start a new message
            if(serial_port_status == AWATING_DATA)
            {
                serial_port_status = GETTING_DATA;
                
                memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
                serial_buffer[0] = newByte;
                serial_buffer_size = 1;
            }
            // Otherwise this is an error!
            else
            {
                 if(debug) Serial.println("[ERROR] Got start byte in the middle of a message!");
                 memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
                 serial_port_status = AWATING_DATA;
            }
        }
        // Got an end byte and a message is being constructed
        else if(newByte == END && serial_port_status == GETTING_DATA)
        {
            if(serial_buffer_size < SERIAL_BUFFER_SIZE-1)
            {
                serial_buffer[serial_buffer_size] = newByte;
                serial_buffer_size++;
                
                serial_buffer[serial_buffer_size] = '\0';
                serial_buffer_size++;
                
                if(debug)
                {
                    debug_msg = String("[INFO] Got message: ") + String(serial_buffer);
                    Serial.println(debug_msg);
                    //Serial.flush();
                }
                
                serial_port_status = AWATING_DATA;             
                
                int action = getValue(1);
                
                if(action == -1 && debug){ 
                  Serial.println("[ERROR] Could not find an action in the message!"); 
                }
                
                if(action > 0 && action < ACTION_COUNT)
                {
                    for(int i=0 ; i<ACTION_PARAM_COUNT[action-1] ; i++)
                    {
                        argv[i] = getValue(next_separator+1);
                        
                        if(argv[i] == -1)
                        {
                            if(debug) Serial.println("[ERROR] Insufficient number of parameters for this action!");
                            return 0;
                        }
                    }
                    //Serial.println(action);
                    return action;
                }
                else
                {
                    if(debug)
                    {
                        debug_msg = "[ERROR] Unknown action: "; 
                        debug_msg += String(action, DEC);
                        Serial.println(debug_msg);
                    }
                }
            }
            else
            {
                 if(debug) Serial.println("[ERROR] Buffer size exceeded!");
                 memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
                 serial_port_status = AWATING_DATA;
            }
        }
        // Got a byte and a message is being constructed
        else if(serial_port_status == GETTING_DATA)
        {
            if(serial_buffer_size < SERIAL_BUFFER_SIZE)
            {
                serial_buffer[serial_buffer_size] = newByte;
                serial_buffer_size++;
            }
            else
            {
                 if(debug) Serial.println("[ERROR] Buffer size exceeded!");
                 memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
                 serial_port_status = AWATING_DATA;
            }
        }
    }
    return 0;
}
    
void RobotSerialComm::reply(unsigned int action, unsigned int * argv, int argc)
{
    char str_param [10]; 
    int i;
    
    Serial.print(START);
    itoa(action,str_param,10);    
    Serial.print(str_param);
    
    for(i=0 ; i<argc ; i++){       
      Serial.print(SEPARATOR);
       itoa(argv[i],str_param,10);        
      Serial.print(str_param);
    }
    
    Serial.println(END);    
    
}

// Helper function for retrieving int values from the serial input buffer
int RobotSerialComm::getValue(int start_index)
{
    char data[8];
    int i = 0;
    
    while(serial_buffer[start_index+i] != SEPARATOR && serial_buffer[start_index+i] != END && start_index+i < serial_buffer_size-1 && i < 8) //ainda falha qd ha params a menos (ex: 2 em vez de 3) 
    {
        data[i] = serial_buffer[start_index+i];
        i++;
    }
    data[i] = '\0';
    next_separator = start_index+i;
    
    if(i == 0) return -1;

    return atoi(data);
}

void RobotSerialComm::sendDebugMsg()
{
    if(debug) Serial.println(debug_msg);
}

int RobotSerialComm::getDebug()
{
    if(debug) Serial.println("[INFO] Debug mode is ON!");
    else Serial.println("[INFO] Debug mode is OFF...");
    return debug;
}

void RobotSerialComm::setDebug(int d)
{
    debug = d;    
    if(debug) Serial.println("[INFO] Debug mode is ON!");
    else Serial.println("[INFO] Debug mode is OFF...");
}

// EOF
