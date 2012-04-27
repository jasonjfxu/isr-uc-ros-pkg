/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 List of available routines to control Omni3MD board;
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
 on 28 April 2011 and Update by José Cruz (www.botnroll.com)
 on 18 May 2011
 
 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   //required by Omni3MD.cpp
#include <Omni3MD.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address

Omni3MD myOmni;                     //declaration of object variable to control the Omni3MD
//variable declarations
byte linear_speed=0;                //linear speed varies from 0 (stopped) to 100 (max speed)
byte rotational_speed=100;          //rotation varies from 0 (CCW) to 200 (CW). At 100 it doesn't rotate.
int direc=0;                        //direction varies from 0 (front) to 360 degrees.

byte way1=1;                        //motor 1 will run CW by default
byte speed1=0;                      //motor 1 speed is initially zero
byte way2=1;                        //motor 2 will run CW by default
byte speed2=0;                      //motor 2 speed is initially zero
byte way3=1;                        //motor 3 will run CW by default
byte speed3=0;                      //motor 3 speed is initially zero

float myOmniTemperature=0x00;
float myOmniBattery=0x00;
float myOmniFirmware=0x00;
byte read_control_rate=0x00;
int read_enc_lim1=0x00;
int read_enc_lim2=0x00;
int read_enc_lim3=0x00;
int read_enc_max1=0x00;
int read_enc_max2=0x00;
int read_enc_max3=0x00;
int read_enc1=0x00;
int read_enc2=0x00;
int read_enc3=0x00;

byte control_time=100;

void setup()
{
    //setup routines
    myOmni.i2c_connect(OMNI3MD_ADDRESS);  //set i2c connection
    delay(10);                      // pause 10 milliseconds
    myOmni.set_way(0x01);           // set the correct way value according to the motor gearbox [byte way] (requires 10ms)
    delay(10);                      // pause 10 milliseconds
    myOmni.calibrate_omni();        // send the calibration comand to configure the OMmni3MD board (requires 10s)
    delay(11000);                   // wait 11s for calibration to end
    myOmni.stop_motors();           // stop all motors
    delay(10);                      // pause (requires 10ms)
    myOmni.set_i2c_timeout (0x00);  // [byte timeout] x 100 miliseconds to receive i2C Commands
    delay(10);
    myOmni.set_i2c_address (OMNI3MD_ADDRESS); // [byte newAddress]
    delay(10);
    myOmni.set_PID(80,20,5);         //[word Kp, word Ki, word Kd] (requires 10ms)
    delay(10);
    
    //set_prescaler(byte encoder, byte value)  value: 0 - 1 pulse; 1 - 10 pulses; 2 - 100 pulses; 3 - 1000 pulses; 4 - 10000 pulses  (requires 10ms)
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
    myOmni.set_enc_value(3,0);      // resets to zero the encoder value [byte encoder, word encValue]
    delay(3);
    
    //reading routines
    /*
    myOmniTemperature=myOmni.read_temperature();  //read Temperature Value
    myOmniBattery=myOmni.read_battery();          //read Battery Value
    myOmniFirmware=myOmni.read_firmware();        //read Firmware version
    read_control_rate=myOmni.read_control_rate();
    read_enc_lim1=myOmni.read_enc_lim1();
    read_enc_lim2=myOmni.read_enc_lim2();
    read_enc_lim3=myOmni.read_enc_lim3();
    read_enc_max1=myOmni.read_enc_max1();
    read_enc_max2=myOmni.read_enc_max2();
    read_enc_max3=myOmni.read_enc_max3();
    read_enc1=myOmni.read_enc1();
    read_enc2=myOmni.read_enc2();
    read_enc3=myOmni.read_enc3();
    */
    
    //movement routines
    /*
    myOmni.mov_incremental(1,1,30,500,1);   // move motor 1, CW at speed 30% until encoder reaches 500 counts and then brake [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
    myOmni.mov_incremental(2,1,30,500,0);   // move motor 2, CW at speed 30% until encoder reaches 500 counts and then set motor loose [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
    myOmni.mov_incremental(3,2,50,2000,0);  // move motor 3, CCW at speed 50% until encoder reaches 2000 counts and then set motor loose [byte motor,byte way, byte speed,word encPosition,byte stoptorque]
    myOmni.mov_omni_pid(linear_speed,rotational_speed,direc);     //[byte linear_speed,byte rotational_speed,word direction]
    myOmni.mov_omni_nopid(linear_speed,rotational_speed,direc);   //[byte linear_speed,byte rotational_speed,word direction]
    myOmni.mov_lin3M_pid(way1,speed1,way2,speed2,way3,speed3);      //[byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3]
    myOmni.mov_lin3M_nopid(way1,speed1,way2,speed2,way3,speed3);    //[byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3]
    myOmni.mov_lin1M_pid(1,1,60);                                   //[byte motor,byte way, byte speed]
    myOmni.mov_lin1M_nopid(1,1,100);                                //[byte motor,byte way, byte speed]
    */
}
void loop()
{
}
