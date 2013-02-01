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
* Originally by: Gonçalo Cabrita on 22/09/2011
* Modified by:   Andre Araujo & David Portugal on 04/01/2013
*********************************************************************/

#include "Arduino.h"

#define SERIAL_BUFFER_SIZE  32

#define START            '@'
#define SEPARATOR        ','
#define END              'e'

enum RobotAction
{
  
  // Setup routines
  OMNI_CALIBRATION = 1,
  OMNI_SET_PID = 2,
  OMNI_SET_PRESCALER = 3,
  OMNI_SET_ENC_VALUE = 4,
    
  // Reading routines
  ROBOT_INFO = 5,
  OMNI_READ_ENCODERS = 6,
  READ_SONARS = 7,
  READ_ENCODERS_SONARS = 8,
 
  // Movement routines
  LINEAR_MOVE_PID = 9,
  LINEAR_MOVE_NOPID = 10,
  MOVE_DIFFERENTIAL_SI = 11,
  MOVE_POSITIONAL = 12,
  STOP_MOTORS = 13,
  ENCODERS_RESET = 14,
  
  // COMM
  ACTION_GET_DEBUG = 15,
  ACTION_SET_DEBUG = 16,
  ACTION_GET_STREAM = 17,
  ACTION_START_STREAM = 18,
  ACTION_STOP_STREAM = 19,
  ACTION_COUNT = 20

};

extern int ACTION_PARAM_COUNT[];

enum
{
    AWATING_DATA = 0,
    GETTING_DATA = 1
};

class RobotSerialComm
{
    public:
    RobotSerialComm();
    
    int getMsg(unsigned int * argv);
    
    void reply(unsigned int action, unsigned int * argv, int argc);
    
    void sendDebugMsg();
    int getDebug();
    void setDebug(int d);
    
    // Debug message buffer
    String debug_msg;
    // Debug ON (1) or OFF (0)
    byte debug;
    
    private:
    // Buffer for serial input
    char serial_buffer[SERIAL_BUFFER_SIZE];
    // The current size of serial input buffer
    int serial_buffer_size;
    // Status of the serial port, AWATING_DATA or GETTING_DATA
    byte serial_port_status;
    
    // Used by the function getValue to store the index of the next separator byte
    int next_separator;
    
    int getValue(int start_index);
};

// EOF

