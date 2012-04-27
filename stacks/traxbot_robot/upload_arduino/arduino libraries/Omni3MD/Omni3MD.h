/*
  Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
  Created by Nino Pereira, April 28, 2011.
  Released into the public domain.
*/

#ifndef Omni3MD_h
#define Omni3MD_h

#include "WProgram.h"

#define KEY1 0xAA // key used in critical commands
#define KEY2 0x55 // key used in critical commands

/*User Commands*/
/*Read Firmware version*/
#define COMMAND_FIRMWARE_INT       0xFE //Parte inteira da versão do firmware
#define COMMAND_FIRMWARE_DEC       0xFD //Parte decimal da versão do firmware
/*Write Commands->Don't require response from Omni3MD */
#define COMMAND_STOP               0xFC //Comando de paragem sem controlo
#define COMMAND_CALIBRATE          0xFB //Calibração dos parâmetros físicos do movimento
#define COMMAND_MOV_OMNI_PID       0xFA //Comando de movimento omidireccional com controlo PID
#define COMMAND_MOV_OMNI_NOPID     0xF9 //Comando de movimento omidireccional sem controlo
#define COMMAND_MOV_LIN3M_PID      0xF8 //Comando de movimento linear de 3 motores com controlo PID
#define COMMAND_MOV_LIN3M_NOPID    0xF7 //Comando de movimento linear de 3 motores sem controlo PID
#define COMMAND_MOV_LIN1M_PID      0xF6 //Comando de movimento linear de 1 motor com controlo PID
#define COMMAND_MOV_LIN1M_NOPID    0xF5 //Comando de movimento linear de 1 motor sem controlo PID
#define COMMAND_GAIN_UPDATE        0xF4 //Recepção dos ganhos para o controlo
#define COMMAND_CFG_MOV_CW         0xF3 //Define o sentido CW do movimento
#define COMMAND_I2C_ADD            0xF2 //Comando para mudança do Endereço I2C
#define COMMAND_TIMEOUT_I2C        0xF1 //Comando para configuração do tempo de timeout I2C
#define COMMAND_PRESCALER_CFG      0xF0 //Comando para configuração do presclaer da contagem incremental dos encoders
#define COMMAND_INC_ENC_PRESET     0xEF //Comando para efectuar reset à contagem incremental dos encoders
#define COMMAND_MOV_INC            0xEE //Comando de movimento linear incremental de um motor

/*Read Commands-> requests to Omni3MD */
#define COMMAND_BAT_HI        	  0xE4 //Leitura do valor da temperatura da placa
#define COMMAND_BAT_LOW       	  0xE3 //Leitura do valor da tensão da bateria
#define COMMAND_TEMPER1_HI        0xE2 //Leitura do valor da temperatura da placa (sensor1)
#define COMMAND_TEMPER1_LOW       0xE1 //Leitura do valor da temperatura da placa (sensor1)
#define COMMAND_ENC1_LIM_HI       0xE0 //PWM no Limiar do movimento para o motor 1
#define COMMAND_ENC1_LIM_LOW      0xDF //Valor obtido na rotina de calibra��o
#define COMMAND_ENC2_LIM_HI       0xDE //PWM no Limiar do movimento para o motor 2
#define COMMAND_ENC2_LIM_LOW      0xDD //Valor obtido na rotina de calibra��o
#define COMMAND_ENC3_LIM_HI       0xDC //PWM no Limiar do movimento para o motor 3
#define COMMAND_ENC3_LIM_LOW      0xDB //Valor obtido na rotina de calibra��o
#define COMMAND_ENC1_MAX_HI       0xDA //Valor de contagens máximo registado para o encoder1
#define COMMAND_ENC1_MAX_LOW      0xD9 //Permite ao utilizador obter o estado do encoder 1
#define COMMAND_ENC2_MAX_HI       0xD8 //Valor de contagens máximo registado para o encoder2
#define COMMAND_ENC2_MAX_LOW      0xD7 //Permite ao utilizador obter o estado do encoder 2
#define COMMAND_ENC3_MAX_HI       0xD6 //Valor de contagens máximo registado para o encoder3
#define COMMAND_ENC3_MAX_LOW      0xD5 //Permite ao utilizador obter o estado do encoder 3
#define COMMAND_CTRL_RATE         0xD4 //Leitura da taxa de controlo dos motores
#define COMMAND_ENC1_INC_HI       0xD3 //Contagem incremental do encoder 1 HI
#define COMMAND_ENC1_INC_LOW      0xD2 //Contagem incremental do encoder 1 LO
#define COMMAND_ENC2_INC_HI       0xD1 //Contagem incremental do encoder 2 HI
#define COMMAND_ENC2_INC_LOW      0xD0 //Contagem incremental do encoder 2 LO
#define COMMAND_ENC3_INC_HI       0xCF //Contagem incremental do encoder 3 HI
#define COMMAND_ENC3_INC_LOW      0xCE //Contagem incremental do encoder 3 LO

#include "WProgram.h"

class Omni3MD
{
  public:
    //Omni3MD(byte omniAddress);

    //setup routines
    void i2c_connect(byte omniAddress);
    void calibrate_omni();
    void set_i2c_timeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
    void set_i2c_address (byte newAddress);
//    void set_way(byte way);
    void set_PID(word Kp, word Ki, word Kd);
    void set_prescaler(byte encoder, byte value);
    void set_enc_value(byte encoder, word encValue);
	
    //reading routines
    float read_temperature();
    float read_battery();
    float read_firmware();
//    byte read_control_rate();
//    int read_enc_lim1();
//    int read_enc_lim2();
//    int read_enc_lim3();
//    int read_enc_max1();
//    int read_enc_max2();
//    int read_enc_max3();
    int read_enc1();
    int read_enc2();
//    int read_enc3();

	//movement routines
//    void mov_omni_pid(byte linear_speed,byte rotational_speed,word direction);
//    void mov_omni_nopid(byte linear_speed,byte rotational_speed,word direction);
    void mov_lin3M_pid(byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3);
//    void mov_lin3M_nopid(byte way1,byte speed1,byte way2,byte speed2,byte way3,byte speed3);
//    void mov_lin1M_pid(byte motor,byte way, byte speed);
//    void mov_lin1M_nopid(byte motor,byte way, byte speed);
//    void mov_incremental(byte motor,byte way, byte speed,word encPosition,byte stoptorque);
    void stop_motors();
    
  private:
    byte _omniAddress;
    byte i2cRequestData(byte addressValue, byte command);
    void i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes);
};

#endif




