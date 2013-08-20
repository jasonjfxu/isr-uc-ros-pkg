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
*  Author: Nuno Ferreira on 15/05/2013
*********************************************************************/

#include "MRsensing.h"
#include <Wire.h>



unsigned long time_rise=0;
unsigned long time_fall=0;
unsigned long time_mus=0;
unsigned int ov_counter=0;
int quantity=0;//2011/12/29 change by bruce
float time_sum=0;
float rate=0;

void MRsensing::sensorSetup() {
  
  //set 16bit counter for measure the wide of sensor
  TCCR1A = 0;
  TCCR1B |=1<<(CS12)|0<<(CS11)|1<<(CS10);//Set clock 1024/16MHz,unit is 6.4us
  TIMSK1 |=1<<(ICIE1)|1<<(TOIE1); //enable capture interrupt and overflow interrupt
  TCNT1 = 0;
  delay(3000);
  
  Wire.begin();        // join i2c bus (address optional for master) 
  
  pinMode(DUST_PIN,INPUT);
  pinMode(ALCOHOL_SelPin,OUTPUT);    // set the heaterSelPin as digital output.
  pinMode(fan_pin,OUTPUT);
  digitalWrite(ALCOHOL_SelPin,HIGH); //when heaterSelPin is set, heater is switched off.
  digitalWrite(fan_pin,HIGH);
  
  sei();//enable interrupt
}


int MRsensing::getLDRsensor() {
  
  int LDRsensorValue = 0;
  LDRsensorValue = analogRead(LDR_Pin);
  return LDRsensorValue;
  
}



int MRsensing::getAlcoholSensor() {
  
  int sensorValue = 0;
  digitalWrite(ALCOHOL_SelPin,LOW);                //switch on the heater of Alcohol sensor
  sensorValue = analogRead(ALCOHOL_InDatPin);       //read the analog value
  sensorValue = 1023 - sensorValue; 

  return sensorValue;
}


int MRsensing::getDustSensor(){
  return quantity;
}



void MRsensing::getThermopileSensor(int thermopile_tab[]) {
  
  int idx=0;
  
  for (idx=1; idx<=9; idx++) {
    
    Wire.beginTransmission(TPA81ADDR);
    Wire.write(idx);
    Wire.endTransmission();
    Wire.requestFrom(TPA81ADDR,  1);
    while(Wire.available() < 1) {     // Wait for incoming idx thermopile frame
    }

    thermopile_tab[idx-1]= Wire.read(); // receive a byte as character 
  }
  
}

int MRsensing::getBearing(){
  
   byte highByte, lowByte, fine;              // highByte and lowByte store high and low bytes of the bearing and fine stores decimal place of bearing
   char pitch, roll;                          // Stores pitch and roll values of CMPS10, chars are used because they support signed value
   int bearing;                               // Stores full bearing
   
   Wire.beginTransmission(CMPS10);           //starts communication with CMPS10
   Wire.write(2);                              //Sends the register we wish to start reading from
   Wire.endTransmission();

   Wire.requestFrom(CMPS10, 4);              // Request 4 bytes from CMPS10
   while(Wire.available() < 4);               // Wait for bytes to become available
   highByte = Wire.read();           
   lowByte = Wire.read();            
   pitch = Wire.read();              
   roll = Wire.read();               
   
   bearing = ((highByte<<8)+lowByte)/10;      // Calculate full bearing
   fine = ((highByte<<8)+lowByte)%10;         // Calculate decimal place of bearing
  
  return bearing;
  
}

//duty measure
ISR(TIMER1_OVF_vect)
{
	if(ov_counter==7)
		{
			PORTD^=0x40;
			ov_counter=0;
			//Serial.println(time_sum);
			rate=(float)(time_sum/336000);
			if(rate<=8)
				{
					quantity=rate*562.5;//8 equal 4500 pcs Particle according to the datasheet.
					}
			else
				quantity=4500+(rate-8)*750;
 
                        //Serial.print("quantity is :");
                        //Serial.println(quantity);
			//Serial.print("rate is :");
			//Serial.println(rate,8);
			time_sum=0;
			}
	else
		{
			ov_counter++;
			//digitalWrite(6,HIGH);			
			//Serial.println(ov_counter);
			}
	}
 
 
ISR(TIMER1_CAPT_vect)
{
 
	if((PORTB^0x01)==1)
		{
			//time_fall=ICR1;	
			time_fall=micros();			
			TCCR1B=0x45; //change to rising capture and with 1024 prescaler
			digitalWrite(13,HIGH);
			//TIFR1|=1<<(TOV1);//reset the flag
			}
	else
		{
			time_rise=micros();
			TCCR1B=0x05; //change to negative and with 1024 prescaler
			digitalWrite(13,LOW);
			//TIFR1|=1<<(TOV1);//reset the flag
			if(time_rise>time_fall)
			time_mus=20000+(time_rise-time_fall);//20000 is countervail for program run
			time_sum+=+time_mus;
		}		
 
};
