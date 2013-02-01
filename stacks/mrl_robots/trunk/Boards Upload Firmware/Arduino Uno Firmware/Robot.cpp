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

#include "Arduino.h"
#include "Robot.h"
#include <Wire.h>

Omni3MD omni3md;

void Robot::robotSetup() {
}

int Robot::getRange(int pin_num) {

  int aux_sensorValue=0, sensorValue=0, cnt=0;
  int samples = 20;

  delay(10);

  while (cnt != samples) {
     digitalWrite(12, HIGH);					// Activate chain reading on sonars
     sensorValue = analogRead(pin_num);
     sensorValue = 1.767 * pow(sensorValue,0.9465) - 0.3759;     // Convert Scale
     aux_sensorValue = aux_sensorValue + sensorValue;
     cnt = cnt + 1;
     digitalWrite(12, LOW);
   }
   cnt=0;
   sensorValue = round( aux_sensorValue / samples); 
  
   return sensorValue;
   
}

void Robot::encodersReset(){     // This function resets the encoder values to 0
 
  omni3md.set_enc_value(1,0);   // presets the encoder value [byte encoder, word encValue]
  omni3md.set_enc_value(3,0);   // presets the encoder value [byte encoder, word encValue]

}

// Write and Read Robot ID to the EEPROM:

void Robot::EEPROMWriteDouble(int ee, double value){
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
	  EEPROM.write(ee++, *p++);
}

double Robot::EEPROMReadDouble(int ee){
    double value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return value;
}







