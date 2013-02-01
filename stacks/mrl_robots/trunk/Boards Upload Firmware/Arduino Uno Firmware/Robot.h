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
*  Author: Andre Araujo on 19/12/2012
*********************************************************************/

#include <EEPROM.h>
#include "Arduino.h"
#include "Omni3MD.h"

#define ROBOT_FIRMWARE_VERSION 1

#define BAUD_RATE 115200//19200
#define OMNI3MD_ADDRESS 0x30        // default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address
#define FRONT_SONAR 0
#define LEFT_SONAR  1
#define RIGHT_SONAR 2
#define Kp 700                    // PID Gain
#define Ki 400                    // PID Gain
#define Kd 200                    // PID Gain
#define gearbox_factor 43.7
#define encoder_cpr 64



class Robot
{
  public:
  void robotSetup();
  int getRange(int pin_num);
  void encodersReset();
  void EEPROMWriteDouble(int ee, double value);
  double EEPROMReadDouble(int ee);
  
  
  private:
  
};
