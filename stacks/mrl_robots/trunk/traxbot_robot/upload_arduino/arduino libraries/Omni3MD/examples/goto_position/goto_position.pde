/*
 Independent 3 motor position control using Omni3MD (www.botnroll.com)
 Demonstrates how to control the position of up to three independent motors using Omni3MD;
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

int read_enc1=0x00;
int read_enc2=0x00;
int read_enc3=0x00;

void debug()
{
  Serial.print("Temperature:");
  Serial.println(myOmniTemperature);    // prints temperature Value
  Serial.print("Battery:");
  Serial.println(myOmniBattery);        // prints battery Value
  Serial.print("Firmware:");
  Serial.println(myOmniFirmware);       // prints firmware Value
}

void debug_enc()
{
  Serial.print("Encoders pos:");
  Serial.print(read_enc1);
  Serial.print(",");
  Serial.print(read_enc2);
  Serial.print(",");
  Serial.println(read_enc3);
}
void setup()
{
    //setup routines
    Serial.begin(9600);             // set baud rate to 9600 bps
    myOmni.i2c_connect(OMNI3MD_ADDRESS); //set i2c connection
    delay(10);                        // pause 10 milliseconds
    myOmni.set_way(0x01);           // set the correct way value according to the motor gearbox [byte way]
    delay(10);                     // pause 10 milliseconds
//    myOmni.calibrate_omni();        // send the calibration comand to configure the OMmni3MD oard
//    delay(11000);                   // wait 11s for calibration to end
    myOmni.stop_motors();           // stops all motors
    delay(10);
    
    myOmni.set_i2c_timeout (0);  // [byte timeout] x 100 miliseconds to receive i2C Commands; 0=no timeout
    delay(10);
    myOmni.set_PID(40,10,5);        // set PID parameters Kp,Ki and Kd [word Kp, word Ki, word Kd]
    delay(10);
    //reading routines
    myOmniTemperature=myOmni.read_temperature();  //read Temperature Value
    myOmniBattery=myOmni.read_battery();          //read Battery Value
    myOmniFirmware=myOmni.read_firmware();        //read Firmware version
    
    debug();                        // print debug information

    //set_prescaler(byte encoder, byte value)  value: 0 - 1 pulse; 1 - 10 pulses; 2 - 100 pulses; 3 - 1000 pulses; 4 - 10000 pulses
    myOmni.set_prescaler(1, 2);     //sets the prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
    delay(10);
    myOmni.set_prescaler(2, 2);     //sets the prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
    delay(10);
    myOmni.set_prescaler(3, 2);     //sets the prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
    delay(10);
    myOmni.set_enc_value(1,0);      // resets to zero the encoder value [byte encoder, word encValue]
    delay(3);
    myOmni.set_enc_value(2,0);      // resets to zero the encoder value [byte encoder, word encValue]
    delay(3);
    myOmni.set_enc_value(3,4000);      // resets to zero the encoder value [byte encoder, word encValue]
    delay(3);
    // send movement instructions    
    myOmni.mov_incremental(1,1,100,500,1);   // move motor 1, CW at speed 30% until encoder reaches 500 counts and then brake [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
    myOmni.mov_incremental(2,1,100,500,1);   // move motor 2, CW at speed 30% until encoder reaches 500 counts and then set motor loose [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
    myOmni.mov_incremental(3,2,100,3500,1);  // move motor 3, CCW at speed 50% until encoder reaches 2000 counts and then set motor loose [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
}

void loop()
{
   read_enc1=myOmni.read_enc1();      // read encoder 1
   read_enc2=myOmni.read_enc2();      // read encoder 2
   read_enc3=myOmni.read_enc3();      // read encoder 3
   debug_enc();                       // print encoder count
   delay(1000);                       // pause 1s
}
