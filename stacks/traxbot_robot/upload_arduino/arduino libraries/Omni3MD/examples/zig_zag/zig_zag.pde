/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 Demonstrates how to produce a zig-zag movement with using Omni3MD;
 - this code assumes you are using 3 DC motors with encoders attached to Omni3MD board;

 Omni3MD board features: 
 - Holonomic movement integrated control of 3 motors with or without PID control;
 - Linear control of 3 independent motors with or without PID control;
 - Independent position control using encoder readings;
 - Encoders readings;
 - Battery reading;
 - Temperature reading;
 
 The circuit:
 * Omni3MD board attached to Arduino analog input 4 (SDA) and 5 (SCL), GND and 5V DC.
 
 This example was created by Nino Pereira (www.botnroll.com)
 on 28 April 2011 and updated by José Cruz (www.botnroll.com)
 on 18 May 2011
 
 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   // required by Omni3MD.cpp
#include <Omni3MD.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        // default factory address

Omni3MD myOmni;    // [byte omniAddress]

//variable declarations
byte linear_speed=0;                // linear speed varies from 0 (stopped) to 100 (max speed)
byte rotational_speed=100;          // rotation varies from 0 (CCW) to 200 (CW). At 100 it doesn't rotate.
int direc=0;                        // direction varies from 0 (front) to 359 degrees.

float myOmniTemperature=0;
float myOmniBattery=0;
float myOmniFirmware=0;

int control_time=100;

void debug()
{
  Serial.print("Temperature:");
  Serial.println(myOmniTemperature);    // prints temperature Value
  Serial.print("Battery:");
  Serial.println(myOmniBattery);        // prints battery Value
  Serial.print("Firmware:");
  Serial.println(myOmniFirmware);       // prints firmware Value
  Serial.print("Control Time:");
  Serial.println(control_time);
}

void setup()
{
    //setup routines
    Serial.begin(9600);             // set baud rate to 9600 bps
    myOmni.i2c_connect(OMNI3MD_ADDRESS); //set i2c connection
    delay(10);                        // pause 10 milliseconds
    myOmni.set_way(0x01);           // set the correct way value according to the motor gearbox [byte way]
    delay(10);                     // pause 10 milliseconds
   // myOmni.calibrate_omni();        // send the calibration comand to configure the OMmni3MD board
    //delay(11000);                   // wait 11s for calibration to end
    myOmni.stop_motors();           // stops all motors
    delay(10);
    control_time=(1000/myOmni.read_control_rate())*2;
    
    myOmni.set_i2c_timeout (control_time/20);  // [byte timeout] x 100 miliseconds to receive i2C Commands or 0x00 for no timeout
    delay(10);
    myOmni.set_PID(40,40,20);        // set PID parameters Kp, Ki and Kd [word Kp, word Ki, word Kd]
    delay(10);

    //reading routines
    myOmniTemperature=myOmni.read_temperature();  //read Temperature Value
    myOmniBattery=myOmni.read_battery();          //read Battery Value
    myOmniFirmware=myOmni.read_firmware();        //read Firmware version
    
    debug();                        // print debug information
}

void loop()
{
  linear_speed=50;                                                 //20% of maximum speed                                                     //direction forward
  rotational_speed= 100;
  
  for ( direc=90,linear_speed=0; linear_speed < 50; linear_speed+=10)
  {
     myOmni.mov_omni_pid(linear_speed,rotational_speed,direc);     //[byte linear_speed,byte rotational_speed,word direction]
     delay(control_time);                                          //pause
  }
  for ( direc=90,linear_speed=50; linear_speed > 0; linear_speed-=10)
  {
     myOmni.mov_omni_pid(linear_speed,rotational_speed,direc);     //[byte linear_speed,byte rotational_speed,word direction]
     delay(control_time);                                          //pause
  }
  for ( direc=270,linear_speed=0; linear_speed < 50; linear_speed+=10)
  {
     myOmni.mov_omni_pid(linear_speed,rotational_speed,direc);     //[byte linear_speed,byte rotational_speed,word direction]
     delay(control_time);                                          //pause
  }
  for ( direc=270,linear_speed=50; linear_speed > 0; linear_speed-=10)
  {
     myOmni.mov_omni_pid(linear_speed,rotational_speed,direc);     //[byte linear_speed,byte rotational_speed,word direction]
     delay(control_time);                                          //pause
  }
}
