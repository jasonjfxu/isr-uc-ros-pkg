/*
  Omni3MD.cpp - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
  Created by Nino Pereira, April 28, 2011.
  Released into the public domain.
*/


#include "Wire.h"
#include "Omni3MD.h"

/*Omni3MD::Omni3MD(byte omniAddress)
{
  _omniAddress = omniAddress;
}*/

///////////////////////////////////////////////////////////////////////
//private routines
///////////////////////////////////////////////////////////////////////
void Omni3MD::i2c_connect(byte omniAddress)
{
    _omniAddress = omniAddress>>1;
	Wire.begin(_omniAddress);						// join i2c bus (address optional for master)
}

byte Omni3MD::i2cRequestData(byte addressValue, byte command)
{
  byte value=(byte)0xFF;
  Wire.beginTransmission(addressValue);				 // transmit to device
  Wire.send(command);								 // sends one byte
  Wire.endTransmission();							 // stop transmitting

  Wire.requestFrom((int)addressValue,(int)1);        // requests one byte
  if (Wire.available())
  {
    value=Wire.receive();
  }
  return value;
}

void Omni3MD::i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes)
{
  byte value=0;
  Wire.beginTransmission(addressValue); // transmit to device #0x10
  Wire.send(command);        // sends one byte 
  for (int k =0; k< numBytes;k++)
    Wire.send(buffer[k]);        // sends one byte
  Wire.endTransmission(); // stop transmitting 
}

///////////////////////////////////////////////////////////////////////
//setup routines
///////////////////////////////////////////////////////////////////////

void Omni3MD::calibrate_omni()
{
	byte buffer[]={KEY1,KEY2};
	i2cSendData(_omniAddress, COMMAND_CALIBRATE,buffer,sizeof(buffer));
}
void Omni3MD::set_i2c_timeout (byte timeout) //timeout x 100 miliseconds to receive i2C Commands
{
  byte buffer[]={timeout, KEY1, timeout, KEY2}; 
  i2cSendData(_omniAddress, COMMAND_TIMEOUT_I2C,buffer,sizeof(buffer));
}
void Omni3MD::set_i2c_address (byte newAddress)
{
  byte buffer[]={newAddress, KEY1, newAddress, KEY2};
  i2cSendData(_omniAddress, COMMAND_I2C_ADD,buffer,sizeof(buffer));
  _omniAddress=newAddress;
}
/* void Omni3MD::set_way(byte way)
{
  byte buffer[]={way, KEY1, way, KEY2};
  i2cSendData(_omniAddress, COMMAND_CFG_MOV_CW,buffer,sizeof(buffer));
}*/
void Omni3MD::set_PID(word Kp, word Ki, word Kd)
{
  byte Kp_H=highByte(Kp);
  byte Kp_L=lowByte(Kp);
  byte Ki_H=highByte(Ki);
  byte Ki_L=lowByte(Ki);
  byte Kd_H=highByte(Kd);
  byte Kd_L=lowByte(Kd);
  byte buffer[]={Kp_H,Kp_L,Ki_H,Ki_L,Kd_H,Kd_L};
  i2cSendData(_omniAddress, COMMAND_GAIN_UPDATE, buffer, sizeof(buffer));
}
void Omni3MD::set_prescaler(byte encoder, byte value)
{
  byte buffer[]={encoder,value,KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_PRESCALER_CFG,buffer,sizeof(buffer));
}
void Omni3MD::set_enc_value(byte encoder, word encValue)
{
  byte encValueHIGH=highByte(encValue);
  byte encValueLOW=lowByte(encValue);
  byte buffer[]={encoder,encValueHIGH,encValueLOW,KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_INC_ENC_PRESET,buffer,sizeof(buffer));
}
  
///////////////////////////////////////////////////////////////////////
//movement routines
///////////////////////////////////////////////////////////////////////

/*void Omni3MD::mov_omni_pid(byte linear_speed,byte rotational_speed,word direction)
{
  byte dir_H=highByte(direction);
  byte dir_L=lowByte(direction);
  byte buffer[]={linear_speed,rotational_speed,dir_H,dir_L};
  i2cSendData(_omniAddress, COMMAND_MOV_OMNI_PID,buffer,sizeof(buffer));
}
void Omni3MD::mov_omni_nopid(byte linear_speed,byte rotational_speed,word direction)
{
  byte dir_H=highByte(direction);
  byte dir_L=lowByte(direction);
  byte buffer[]={linear_speed,rotational_speed,dir_H,dir_L};
  i2cSendData(_omniAddress, COMMAND_MOV_OMNI_NOPID,buffer,sizeof(buffer));
}*/
void Omni3MD::mov_lin3M_pid(byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3)
{
  byte buffer[]={way1,speed1,way2,speed2,way3,speed3};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN3M_PID,buffer,sizeof(buffer));
}
/*void Omni3MD::mov_lin3M_nopid(byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3)
{
  byte buffer[]={way1,speed1,way2,speed2,way3,speed3};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN3M_NOPID,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin1M_pid(byte motor,byte way, byte speed)
{
  byte buffer[]={motor,way,speed};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN1M_PID,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin1M_nopid(byte motor,byte way, byte speed)
{
  byte buffer[]={motor,way,speed};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN1M_NOPID,buffer,sizeof(buffer));
}
void Omni3MD::mov_incremental(byte motor,byte way, byte speed,word encPosition,byte stoptorque)
{
  byte encPosHIGH=highByte(encPosition);
  byte encPosLOW=lowByte(encPosition);
  byte buffer[]={ motor, way, speed, encPosHIGH, encPosLOW, stoptorque};
  i2cSendData(_omniAddress, COMMAND_MOV_INC,buffer,sizeof(buffer));
}*/
void Omni3MD::stop_motors() 
{
  byte buffer[]={KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_STOP,buffer,sizeof(buffer));
}	

//////////////////////////////////////////////////////////////////////////////////////
// Readings routines
//////////////////////////////////////////////////////////////////////////////////////

float Omni3MD::read_temperature()
{
  byte temperHigh=i2cRequestData(_omniAddress, COMMAND_TEMPER1_HI);
  byte temperLow=i2cRequestData(_omniAddress, COMMAND_TEMPER1_LOW);
  float temperatureValue=((int)temperHigh*255)+(int)temperLow;
  return (temperatureValue/10);
}
float Omni3MD::read_battery()
{
  byte batteryValueLow=i2cRequestData(_omniAddress, COMMAND_BAT_LOW);
  byte batteryValueHigh=i2cRequestData(_omniAddress, COMMAND_BAT_HI);
 // float batteryValue=(batteryValueHigh<<8)+batteryValueLow;
  return(((batteryValueHigh<<8)+batteryValueLow)/10.0);
}
float Omni3MD::read_firmware()
{
  byte firmI=i2cRequestData(_omniAddress, COMMAND_FIRMWARE_INT);
  byte firmD=i2cRequestData(_omniAddress, COMMAND_FIRMWARE_DEC);
  float firmwareValue=(firmI<<8)+firmD;
  return (firmwareValue/100.0);
}
/*byte Omni3MD::read_control_rate() // returns 10, 20, 40 (times per second control operation)
{
  return i2cRequestData(_omniAddress, COMMAND_CTRL_RATE);
}
int Omni3MD::read_enc_lim1()
{
  byte enc1_lim_hi=i2cRequestData(_omniAddress, COMMAND_ENC1_LIM_HI);
  byte enc1_lim_low=i2cRequestData(_omniAddress, COMMAND_ENC1_LIM_LOW);
  return ((enc1_lim_hi<<8)+enc1_lim_low);
}
int Omni3MD::read_enc_lim2()
{
  byte enc2_lim_hi=i2cRequestData(_omniAddress, COMMAND_ENC2_LIM_HI);
  byte enc2_lim_low=i2cRequestData(_omniAddress, COMMAND_ENC2_LIM_LOW);
  return ((enc2_lim_hi<<8)+enc2_lim_low);
}
int Omni3MD::read_enc_lim3()
{
  byte enc3_lim_hi=i2cRequestData(_omniAddress, COMMAND_ENC3_LIM_HI);
  byte enc3_lim_low=i2cRequestData(_omniAddress, COMMAND_ENC3_LIM_LOW);
  return ((enc3_lim_hi<<8)+enc3_lim_low);
}
int Omni3MD::read_enc_max1()
{
  byte enc1_max_hi=i2cRequestData(_omniAddress, COMMAND_ENC1_MAX_HI);
  byte enc1_max_low=i2cRequestData(_omniAddress, COMMAND_ENC1_MAX_LOW);
  return ((enc1_max_hi<<8)+enc1_max_low);
}
int Omni3MD::read_enc_max2()
{
  byte enc2_max_hi=i2cRequestData(_omniAddress, COMMAND_ENC2_MAX_HI);
  byte enc2_max_low=i2cRequestData(_omniAddress, COMMAND_ENC2_MAX_LOW);
  return ((enc2_max_hi<<8)+enc2_max_low);
}
int Omni3MD::read_enc_max3()
{
  byte enc3_max_hi=i2cRequestData(_omniAddress, COMMAND_ENC3_MAX_HI);
  byte enc3_max_low=i2cRequestData(_omniAddress, COMMAND_ENC3_MAX_LOW);
  return ((enc3_max_hi<<8)+enc3_max_low);
}*/
int Omni3MD::read_enc1()
{
  byte enc1_inc_hi=i2cRequestData(_omniAddress, COMMAND_ENC1_INC_HI);
  byte enc1_inc_low=i2cRequestData(_omniAddress, COMMAND_ENC1_INC_LOW);
  return ((enc1_inc_hi<<8)+enc1_inc_low);
}
int Omni3MD::read_enc2()
{
  byte enc2_inc_hi=i2cRequestData(_omniAddress, COMMAND_ENC2_INC_HI);
  byte enc2_inc_low=i2cRequestData(_omniAddress, COMMAND_ENC2_INC_LOW);
  return ((enc2_inc_hi<<8)+enc2_inc_low);
}
/*int Omni3MD::read_enc3()
{
  byte enc3_inc_hi=i2cRequestData(_omniAddress, COMMAND_ENC3_INC_HI);
  byte enc3_inc_low=i2cRequestData(_omniAddress, COMMAND_ENC3_INC_LOW);
  return ((enc3_inc_hi<<8)+enc3_inc_low);
}*/
