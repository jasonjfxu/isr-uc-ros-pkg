/*
 *  iRobot_OI.cpp
 *  
 *
 *  Created by Gonçalo Cabrita on 19/05/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	Low level communication layer for the iRobot OI
 *
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <time.h>
#include <sys/time.h>
#include <poll.h>

#include "iRobot_OI.h"


// *****************************************************************************
// Constructor
iRobot_OI::iRobot_OI(const char * newSerialPort)
{
	fileDescriptor = -1;	
	strncpy(serialPort, newSerialPort, sizeof(serialPort)-1);
	
	OImode = OI_MODE_OFF;
	
	this->resetOdometry();
	
	encoderCounts[LEFT] = -1;
	encoderCounts[RIGHT] = -1;
	
	lastEncoderCounts[LEFT] = 0;
	lastEncoderCounts[RIGHT] = 0;
	
	numOfPackets = 0;
	sensorPackets = NULL;
	packetsSize = 0;
	
	// Default packets
	OI_Packet_ID defaultPackets[2] = {OI_PACKET_RIGHT_ENCODER, OI_PACKET_LEFT_ENCODER};
	this->setSensorPackets(defaultPackets, 2, OI_PACKET_RIGHT_ENCODER_SIZE + OI_PACKET_LEFT_ENCODER_SIZE);
}


// *****************************************************************************
// Destructor
iRobot_OI::~iRobot_OI()
{
	// Clean up!
}


// *****************************************************************************
// Open the serial port
int iRobot_OI::openSerialPort(bool fullControl)
{
	struct termios term;
	int flags;

	if(fileDescriptor >= 0)
	{
		// Connection already opened!
		return(-1);
	}	
	fflush(stdout);

	// Open it. non-blocking at first, in case there's no roomba
	if((fileDescriptor = open(serialPort, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0)
	{
		// Error opening serial port!
		return(-1);
	}
	if(tcflush(fileDescriptor, TCIFLUSH) < 0)
	{
		// Error
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	if(tcgetattr(fileDescriptor, &term) < 0)
	{
		// Error
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	cfmakeraw(&term);
  
	// Roomba500 series
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);

	if(tcsetattr(fileDescriptor, TCSAFLUSH, &term) < 0)
	{
		// Error
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	if(this->startOI(fullControl) < 0)
	{
		// Error initializing Roomba
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	if(this->getSensorPackets(1000) < 0)
	{
		// Error getting sensor data from Roomba
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	// We know the robot is there so switch to blocking.
	// Kk, we got data, so now set NONBLOCK, and continue
	if((flags = fcntl(fileDescriptor, F_GETFL)) < 0)
	{
		// Error
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	if(fcntl(fileDescriptor, F_SETFL, flags ^ O_NONBLOCK) < 0)
	{
		// Error
		close(fileDescriptor);
		fileDescriptor = -1;
		return(-1);
	}
	return(0);
}


// *****************************************************************************
// Set the mode
int iRobot_OI::startOI(bool fullControl)
{
	unsigned char cmdBuffer[1];
	
	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	cmdBuffer[0] = OI_OPCODE_START;
	
	if(write(fileDescriptor, cmdBuffer, 1) < 0)
	{
		// Error
		return(-1);
	}
	OImode = OI_MODE_PASSIVE;

	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	cmdBuffer[0] = OI_OPCODE_CONTROL;
	
	if(write(fileDescriptor, cmdBuffer, 1) < 0)
	{
		// Error
		return(-1);
	}
	OImode = OI_MODE_SAFE;
	
	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	if(fullControl)
	{
		cmdBuffer[0] = OI_OPCODE_FULL;
		
		if(write(fileDescriptor, cmdBuffer, 1) < 0)
		{
			// Error
			return(-1);
		}
		OImode = OI_MODE_FULL;
	}
	return(0);
}


// *****************************************************************************
// Close the serial port
int iRobot_OI::closeSerialPort()
{
	this->drive(0.0, 0.0);
	
	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	
	if(close(fileDescriptor) < 0)
	{
		// Error
		return(-1);
	}
	else return(0);
}


// *****************************************************************************
// Send an OP code to the roomba
int iRobot_OI::sendOpcode(OI_Opcode code)
{
	unsigned char toSend = code;

	if(write(fileDescriptor, &toSend, 1) < 0)
	{
		// Error
		return(-1);
	}
	return(0);
}


// *****************************************************************************
// Power down the roomba
int iRobot_OI::powerDown()
{
	return sendOpcode(OI_OPCODE_POWER);
}


// *****************************************************************************
// Set the speeds
int iRobot_OI::drive(double linearSpeed, double angularSpeed)
{
	int leftSpeed_mm_s = (int)((linearSpeed-ROOMBA_AXLE_LENGTH*angularSpeed/2)*1e3);	// Left wheel velocity in mm/s
	int rightSpeed_mm_s = (int)((linearSpeed+ROOMBA_AXLE_LENGTH*angularSpeed/2)*1e3);	// Right wheel velocity in mm/s
	
	return this->driveDirect(leftSpeed_mm_s, rightSpeed_mm_s);
}


// *****************************************************************************
// Set the motor speeds
int iRobot_OI::driveDirect(int leftSpeed, int rightSpeed)
{
	// Limit velocity
	int16_t leftSpeed_mm_s = MAX(leftSpeed, -ROOMBA_MAX_LIN_VEL_MM_S);
	leftSpeed_mm_s = MIN(leftSpeed, ROOMBA_MAX_LIN_VEL_MM_S);
	int16_t rightSpeed_mm_s = MAX(rightSpeed, -ROOMBA_MAX_LIN_VEL_MM_S);
	rightSpeed_mm_s = MIN(rightSpeed, ROOMBA_MAX_LIN_VEL_MM_S);
	
	// Compose comand
	unsigned char cmdBuffer[5];
	cmdBuffer[0] = (unsigned char)OI_OPCODE_DRIVE_DIRECT;
	cmdBuffer[1] = (unsigned char)(rightSpeed_mm_s >> 8);
	cmdBuffer[2] = (unsigned char)(rightSpeed_mm_s & 0xFF);
	cmdBuffer[3] = (unsigned char)(leftSpeed_mm_s >> 8);
	cmdBuffer[4] = (unsigned char)(leftSpeed_mm_s & 0xFF);
	
	if(write(fileDescriptor, cmdBuffer, 5) < 0)
	{
		// Error
		return(-1);
	}
	
	return(0);
}


// *****************************************************************************
// Set the motor PWMs
int iRobot_OI::drivePWM(int leftPWM, int rightPWM)
{
	// TODO: Not yet implemented...
	return(-1);
}


// *****************************************************************************
// Set the brushes motors status
int iRobot_OI::brushes()
{
	// TODO: Not yet implemented...
	return(-1);
}

// *****************************************************************************
// Set the brushes motors PWMs
int iRobot_OI::brushesPWM()
{
	// TODO: Not yet implemented...
	return(-1);
}


// *****************************************************************************
// Set the sensors to read
int iRobot_OI::setSensorPackets(OI_Packet_ID * newSensorPackets, int newNumOfPackets, size_t newBufferSize)
{
	if(sensorPackets == NULL)
	{
		delete [] sensorPackets;
	}
	
	numOfPackets = newNumOfPackets;
	sensorPackets = new OI_Packet_ID[numOfPackets];
	
	for(int i=0 ; i<numOfPackets ; i++)
	{
		sensorPackets[i] = newSensorPackets[i];
	}

	packetsSize = newBufferSize;
	return(0);
}


// *****************************************************************************
// Read the sensors
int iRobot_OI::getSensorPackets(int timeout)
{
	struct pollfd ufileDescriptor[1];
	unsigned char cmdBuffer[numOfPackets+2];
	unsigned char dataBuffer[packetsSize];
	
	int retval;
	int numread;
	int totalnumread;
	
	// Fill in the command buffer to send to the robot
	cmdBuffer[0] = OI_OPCODE_QUERY;				// Query
	cmdBuffer[1] = numOfPackets;				// Number of packets
	for(int i=0 ; i<numOfPackets ; i++)
	{
		cmdBuffer[i+2] = sensorPackets[i];		// The packet IDs
	}
	
	if(write(fileDescriptor, cmdBuffer, numOfPackets+2) < 0)
	{
		// Error
		return(-1);
	}
	
	ufileDescriptor[0].fd = fileDescriptor;
	ufileDescriptor[0].events = POLLIN;
	
	totalnumread = 0;
	while(totalnumread < sizeof(dataBuffer))
	{
		retval = poll(ufileDescriptor, 1, timeout);
		
		if(retval < 0)
		{
			if(errno == EINTR)
				continue;
			else
			{
				// Error on poll
				return(-1);
			}
		}
		else if(retval == 0)
		{
			// Error, poll timeout
			return(-1);
		}
		else
		{
			if((numread = read(fileDescriptor, dataBuffer+totalnumread, sizeof(dataBuffer)-totalnumread)) < 0)
			{
				// Error
				return(-1);
			}
			else
			{
				totalnumread += numread;
			}
		}
	}
	
	return this->parseSensorPackets(dataBuffer, (size_t)totalnumread);
}


// *****************************************************************************
// Parse sensor data
int iRobot_OI::parseSensorPackets(unsigned char * buffer , size_t bufferLenght)
{	
	if(bufferLenght != packetsSize)
	{
		// Error wrong packet size
		return(-1);
	}

	int i = 0;
	unsigned int index = 0;
	while(index < packetsSize)
	{
		if(sensorPackets[i]==OI_PACKET_GROUP_0)		// PACKETS 7-26
		{
			index += parseBumpersAndWheeldrops(buffer, index);
			index += parseWall(buffer, index);
			index += parseLeftCliff(buffer, index);
			index += parseFrontLeftCliff(buffer, index);
			index += parseFrontRightCliff(buffer, index);
			index += parseRightCliff(buffer, index);
			index += parseVirtualWall(buffer, index);
			index += parseOvercurrents(buffer, index);
			index += parseDirtDetector(buffer, index);
			index ++;	// Unused byte
			index += parseIrOmniChar(buffer, index);
			index += parseButtons(buffer, index);
			index += parseDistance(buffer, index);
			index += parseAngle(buffer, index);
			index += parseChargingState(buffer, index);
			index += parseVoltage(buffer, index);
			index += parseCurrent(buffer, index);
			index += parseTemperature(buffer, index);
			index += parseBatteryCharge(buffer, index);
			index += parseBatteryCapacity(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_1)		// PACKETS 7-16
		{
		
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_2)		// PACKETS 17-20
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_3)		// PACKETS 21-26
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_4)		// PACKETS 27-34
		{
			index += parseWallSignal(buffer, index);
			index += parseLeftCliffSignal(buffer, index);
			index += parseFrontLeftCliffSignal(buffer, index);
			index += parseFontRightCliffSignal(buffer, index);
			index += parseRightCliffSignal(buffer, index);
			index += 3;	// Unused bytes
			index += parseChargingSource(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_5)		// PACKETS 35-42
		{
		
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_6)		// PACKETS 7-42
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_BUMPS_DROPS)
		{
			index += parseBumpersAndWheeldrops(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_WALL)
		{
			index += parseWall(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_LEFT)
		{
			index += parseLeftCliff(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_FRONT_LEFT)
		{
			index += parseFrontLeftCliff(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_FRONT_RIGHT)
		{
			index += parseFrontRightCliff(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_RIGHT)
		{
			index += parseRightCliff(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_VIRTUAL_WALL)
		{
			index += parseVirtualWall(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_WHEEL_OVERCURRENTS)
		{
			index += parseOvercurrents(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_DIRT_DETECT)
		{
			index += parseDirtDetector(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_IR_CHAR_OMNI)
		{
			index += parseIrOmniChar(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_BUTTONS)
		{
			index += parseButtons(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_DISTANCE)
		{
			index += parseDistance(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_ANGLE)
		{
			index += parseAngle(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CHARGING_STATE)
		{
			index += parseChargingState(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_VOLTAGE)
		{
			index += parseVoltage(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CURRENT)
		{
			index += parseCurrent(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_TEMPERATURE)
		{
			index += parseTemperature(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_BATTERY_CHARGE)
		{
			index += parseBatteryCharge(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_BATTERY_CAPACITY)
		{
			index += parseBatteryCapacity(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_WALL_SIGNAL)
		{
			index += parseWallSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_LEFT_SIGNAL)
		{
			index += parseLeftCliffSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL)
		{
			index += parseFrontLeftCliffSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL)
		{
			index += parseFontRightCliffSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CLIFF_RIGHT_SIGNAL)
		{
			index += parseRightCliffSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_CHARGE_SOURCES)
		{
			index += parseChargingSource(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_OI_MODE)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_SONG_NUMBER)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_SONG_PLAYING)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_STREAM_PACKETS)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_REQ_VELOCITY)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_REQ_RADIUS)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_REQ_RIGHT_VELOCITY)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_REQ_LEFT_VELOCITY)
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_RIGHT_ENCODER)
		{
			index += parseRightEncoderCounts(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LEFT_ENCODER)
		{
			index += parseLeftEncoderCounts(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER)
		{
			index += parseLightBumper(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_LEFT)
		{
			index += parseLightBumperLeftSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_FRONT_LEFT)
		{
			index += parseLightBumperFrontLeftSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_CENTER_LEFT)
		{
			index += parseLightBumperCenterLeftSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_CENTER_RIGHT)
		{
			index += parseLightBumperCenterRightSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_FRONT_RIGHT)
		{
			index += parseLightBumperFrontRightSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LIGHT_BUMPER_RIGHT)
		{
			index += parseLightBumperRightSignal(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_IR_CHAR_LEFT)
		{
			index += parseIrCharLeft(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_IR_CHAR_RIGHT)
		{
			index += parseIrCharRight(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_LEFT_MOTOR_CURRENT)
		{
			index += parseLeftMotorCurrent(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_RIGHT_MOTOR_CURRENT)
		{
			index += parseRightMotorCurrent(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_BRUSH_MOTOR_CURRENT)
		{
			index += parseMainBrushMotorCurrent(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_SIDE_BRUSH_MOTOR_CURRENT)
		{
			index += parseSideBrushMotorCurrent(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_STASIS)
		{
			index += parseStasis(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_100)	// PACKETS 7-58
		{
			
			i++;   
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_101)	// PACKETS 43-58
		{
			index += parseRightEncoderCounts(buffer, index);
			index += parseLeftEncoderCounts(buffer, index);
			index += parseLightBumper(buffer, index);
			index += parseLightBumperLeftSignal(buffer, index);
			index += parseLightBumperFrontLeftSignal(buffer, index);
			index += parseLightBumperCenterLeftSignal(buffer, index);
			index += parseLightBumperCenterRightSignal(buffer, index);
			index += parseLightBumperFrontRightSignal(buffer, index);
			index += parseLightBumperRightSignal(buffer, index);
			index += parseIrCharLeft(buffer, index);
			index += parseIrCharRight(buffer, index);
			index += parseLeftMotorCurrent(buffer, index);
			index += parseRightMotorCurrent(buffer, index);
			index += parseMainBrushMotorCurrent(buffer, index);
			index += parseSideBrushMotorCurrent(buffer, index);
			index += parseStasis(buffer, index);
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_106)	// PACKETS 46-51
		{
			
			i++;
		}
		if(sensorPackets[i]==OI_PACKET_GROUP_107)	// PACKETS 54-58
		{
			
			i++;
		}
	}
	
	//assert(index == bufferLenght != packetsSize);
	
	return(0);
}

int iRobot_OI::parseBumpersAndWheeldrops(unsigned char * buffer, int index)
{
	// Bumps, wheeldrops	
	this->bumper[RIGHT] = (buffer[index]) & 0x01;
	this->bumper[LEFT] = (buffer[index] >> 1) & 0x01;
	this->wheelDrop[RIGHT] = (buffer[index] >> 2) & 0x01;
	this->wheelDrop[LEFT] = (buffer[index] >> 3) & 0x01;
	
	return OI_PACKET_BUMPS_DROPS_SIZE;
}

int iRobot_OI::parseWall(unsigned char * buffer, int index)
{
	// Wall
	this->wall = buffer[index] & 0x01;
	
	return OI_PACKET_WALL_SIZE;
}
	
int iRobot_OI::parseLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff[LEFT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_LEFT_SIZE;
}

int iRobot_OI::parseFrontLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff[FRONT_LEFT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_FRONT_LEFT_SIZE;
}

int iRobot_OI::parseFrontRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff[FRONT_RIGHT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_FRONT_RIGHT_SIZE;
}

int iRobot_OI::parseRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff[RIGHT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_RIGHT_SIZE;
}

int iRobot_OI::parseVirtualWall(unsigned char * buffer, int index)
{
	// Virtual Wall
	this->virtualWall = buffer[index] & 0x01;
	
	return OI_PACKET_VIRTUAL_WALL_SIZE;
}
	
int iRobot_OI::parseOvercurrents(unsigned char * buffer, int index)
{
	// Overcurrent
	unsigned char byte = buffer[index];
	
	this->overcurrent[SIDE_BRUSH] = (byte >> 0) & 0x01;
	this->overcurrent[MAIN_BRUSH] = (byte >> 2) & 0x01;
	this->overcurrent[RIGHT] = (byte >> 3) & 0x01;
	this->overcurrent[LEFT] = (byte >> 4) & 0x01;
	
	return OI_PACKET_WHEEL_OVERCURRENTS_SIZE;
}

int iRobot_OI::parseDirtDetector(unsigned char * buffer, int index)
{
	// Dirt Detector
	this->dirtDetect = buffer[index];

	return OI_PACKET_DIRT_DETECT_SIZE;
}
	
int iRobot_OI::parseIrOmniChar(unsigned char * buffer, int index)
{
	// Infrared Character Omni
	this->irChar[OMNI] = buffer[index];

	return OI_PACKET_IR_CHAR_OMNI_SIZE;
}

int iRobot_OI::parseButtons(unsigned char * buffer, int index)
{
	// Buttons
	for(int i=0 ; i<8 ; i++) this->buttons[i] = (buffer[index] >> i) & 0x01;
	
	return OI_PACKET_BUTTONS_SIZE;
}
	
int iRobot_OI::parseDistance(unsigned char * buffer, int index)
{
	// Distance
	distance = buffer2signed_int(buffer, index);
	
	return OI_PACKET_DISTANCE_SIZE;
}

int iRobot_OI::parseAngle(unsigned char * buffer, int index)
{
	// Angle
	angle = buffer2signed_int(buffer, index);

	return OI_PACKET_ANGLE_SIZE;
}
	
int iRobot_OI::parseChargingState(unsigned char * buffer, int index)
{
	// Charging State
	this->chargingState = buffer[index];
	
	return OI_PACKET_CHARGING_STATE_SIZE;
}

int iRobot_OI::parseVoltage(unsigned char * buffer, int index)
{
	// Voltage
	this->voltage = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_VOLTAGE_SIZE;
}

int iRobot_OI::parseCurrent(unsigned char * buffer, int index)
{
	// Current
	this->current = (float)(buffer2signed_int(buffer, index) / 1000.0);

	return OI_PACKET_CURRENT_SIZE;
}

int iRobot_OI::parseTemperature(unsigned char * buffer, int index)
{
	// Temperature
	this->temperature = (char)(buffer[index]);

	return OI_PACKET_TEMPERATURE_SIZE;
}

int iRobot_OI::parseBatteryCharge(unsigned char * buffer, int index)
{
	// Charge
	this->charge = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CHARGE_SIZE;
}

int iRobot_OI::parseBatteryCapacity(unsigned char * buffer, int index)
{
	// Capacity
	this->capacity = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CAPACITY_SIZE;
}
	
int iRobot_OI::parseWallSignal(unsigned char * buffer, int index)
{
	// Wall signal
	this->wallSignal = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_WALL_SIGNAL_SIZE;
}
	
int iRobot_OI::parseLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliffSignal[LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_LEFT_SIGNAL_SIZE;
}

int iRobot_OI::parseFrontLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliffSignal[FRONT_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL_SIZE;
}
	
int iRobot_OI::parseFontRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliffSignal[FRONT_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL_SIZE;
}

int iRobot_OI::parseRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliffSignal[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_RIGHT_SIGNAL_SIZE;
}	
	
int iRobot_OI::parseChargingSource(unsigned char * buffer, int index)
{
	// Charging soruces available
	int internal = (buffer[index] >> 0) & 0x01;
	int home = (buffer[index] >> 1) & 0x01;
	
	if(internal == 1 && home == 1) this->chargingSource = OI_BOTH_CHARGING_SOURCES;
	else if(internal == 1 && home == 0) this->chargingSource = OI_INTERNAL_CHARGER;
	else if(internal == 1 && home == 0) this->chargingSource = OI_HOME_BASE;
	else if(internal == 0 && home == 0) this->chargingSource = OI_NO_CHARGING_SOURCES;

	return OI_PACKET_CHARGE_SOURCES_SIZE;
}

int iRobot_OI::parseRightEncoderCounts(unsigned char * buffer, int index)
{
	// Right encoder counts
	uint16_t rightEncoderCounts = buffer2unsigned_int(buffer, index);

	//printf("Right Encoder: %d\n", rightEncoderCounts);

	if(encoderCounts[RIGHT] == -1 || rightEncoderCounts == lastEncoderCounts[RIGHT])	// First time, we need 2 to make it work!
	{
		encoderCounts[RIGHT] = 0;
	}
	else
	{
		encoderCounts[RIGHT] = (int)(rightEncoderCounts - lastEncoderCounts[RIGHT]);
		
		if(encoderCounts[RIGHT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoderCounts[RIGHT] = encoderCounts[RIGHT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoderCounts[RIGHT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoderCounts[RIGHT] = ROOMBA_MAX_ENCODER_COUNTS + encoderCounts[RIGHT];
	}
	lastEncoderCounts[RIGHT] = rightEncoderCounts;
	
	return OI_PACKET_RIGHT_ENCODER_SIZE;
}

int iRobot_OI::parseLeftEncoderCounts(unsigned char * buffer, int index)
{
	// Left encoder counts
	uint16_t leftEncoderCounts = buffer2unsigned_int(buffer, index);

	//printf("Left Encoder: %d\n", leftEncoderCounts);

	if(encoderCounts[LEFT] == -1 || leftEncoderCounts == lastEncoderCounts[LEFT])	// First time, we need 2 to make it work!
	{
		encoderCounts[LEFT] = 0;
	}
	else
	{
		encoderCounts[LEFT] = (int)(leftEncoderCounts - lastEncoderCounts[LEFT]);
		
		if(encoderCounts[LEFT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoderCounts[LEFT] = encoderCounts[LEFT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoderCounts[LEFT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoderCounts[LEFT] = ROOMBA_MAX_ENCODER_COUNTS + encoderCounts[LEFT];
	}
	lastEncoderCounts[LEFT] = leftEncoderCounts;
	
	return OI_PACKET_LEFT_ENCODER_SIZE;
}
	
int iRobot_OI::parseLightBumper(unsigned char * buffer, int index)
{
	// Light bumper
	this->irBumper[LEFT] = (buffer[index]) & 0x01;
	this->irBumper[FRONT_LEFT] = (buffer[index] >> 1) & 0x01;
	this->irBumper[CENTER_LEFT] = (buffer[index] >> 2) & 0x01;
	this->irBumper[CENTER_RIGHT] = (buffer[index] >> 3) & 0x01;
	this->irBumper[FRONT_RIGHT] = (buffer[index] >> 4) & 0x01;
	this->irBumper[RIGHT] = (buffer[index] >> 5) & 0x01;
	
	return OI_PACKET_LIGHT_BUMPER_SIZE;
}

int iRobot_OI::parseLightBumperLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_LEFT_SIZE;
}

int iRobot_OI::parseLightBumperFrontLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[FRONT_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_FRONT_LEFT_SIZE;
}

int iRobot_OI::parseLightBumperCenterLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[CENTER_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_CENTER_LEFT_SIZE;
}

int iRobot_OI::parseLightBumperCenterRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[CENTER_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_CENTER_RIGHT_SIZE;
}

int iRobot_OI::parseLightBumperFrontRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[FRONT_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_FRONT_RIGHT_SIZE;
}
	
int iRobot_OI::parseLightBumperRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->irBumperSignal[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_RIGHT_SIZE;
}

int iRobot_OI::parseIrCharLeft(unsigned char * buffer, int index)
{
	// Infrared character left
	this->irChar[LEFT] = buffer[index];

	return OI_PACKET_IR_CHAR_LEFT_SIZE;
}
	
int iRobot_OI::parseIrCharRight(unsigned char * buffer, int index)
{
	// Infrared character left
	this->irChar[RIGHT] = buffer[index];
	
	return OI_PACKET_IR_CHAR_RIGHT_SIZE;
}
	
int iRobot_OI::parseLeftMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motorCurrent[LEFT] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_LEFT_MOTOR_CURRENT_SIZE;
}

int iRobot_OI::parseRightMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motorCurrent[RIGHT] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_RIGHT_MOTOR_CURRENT_SIZE;
}

int iRobot_OI::parseMainBrushMotorCurrent(unsigned char * buffer, int index)
{
	// Main brush motor current
	this->motorCurrent[MAIN_BRUSH] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_BRUSH_MOTOR_CURRENT_SIZE;
}

int iRobot_OI::parseSideBrushMotorCurrent(unsigned char * buffer, int index)
{
	// Main brush motor current
	this->motorCurrent[SIDE_BRUSH] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_SIDE_BRUSH_MOTOR_CURRENT_SIZE;
}

int iRobot_OI::parseStasis(unsigned char * buffer, int index)
{
	// Stasis
	this->stasis = (buffer[index] >> 0) & 0x01;

	return OI_PACKET_STASIS_SIZE;
}

int iRobot_OI::buffer2signed_int(unsigned char * buffer, int index)
{
	int16_t signed_int;
	
	memcpy(&signed_int, buffer+index, 2);
	signed_int = ntohs(signed_int);
	
	return (int)signed_int;
	
}

int iRobot_OI::buffer2unsigned_int(unsigned char * buffer, int index)
{
	uint16_t unsigned_int;

	memcpy(&unsigned_int, buffer+index, 2);
	unsigned_int = ntohs(unsigned_int);
	
	return (int)unsigned_int;
}


// *****************************************************************************
// Calculate Roomba odometry
void iRobot_OI::calculateOdometry()
{	
	double dist = (encoderCounts[RIGHT]*ROOMBA_PULSES_TO_M + encoderCounts[LEFT]*ROOMBA_PULSES_TO_M) / 2.0; 
	double ang = (encoderCounts[RIGHT]*ROOMBA_PULSES_TO_M - encoderCounts[LEFT]*ROOMBA_PULSES_TO_M) / -ROOMBA_AXLE_LENGTH;

	// Update odometry
	this->odometryYaw = NORMALIZE(this->odometryYaw + ang);			// rad
	this->odometryX = this->odometryX + dist*cos(odometryYaw);		// m
	this->odometryY = this->odometryY + dist*sin(odometryYaw);		// m
}


// *****************************************************************************
// Reset Roomba odometry
void iRobot_OI::resetOdometry()
{
	this->setOdometry(0.0, 0.0, 0.0);
}


// *****************************************************************************
// Set Roomba odometry
void iRobot_OI::setOdometry(double newX, double newY, double newYaw)
{
	this->odometryX = newX;
	this->odometryY = newY;
	this->odometryYaw = newYaw;
}


// *****************************************************************************
// Clean
int iRobot_OI::clean()
{
	return sendOpcode(OI_OPCODE_CLEAN);
}


// *****************************************************************************
// Max
int iRobot_OI::max()
{
	return sendOpcode(OI_OPCODE_MAX);
}


// *****************************************************************************
// Spot
int iRobot_OI::spot()
{
	return sendOpcode(OI_OPCODE_SPOT);
}


// *****************************************************************************
// Go to the dock
int iRobot_OI::goDock()
{
	return sendOpcode(OI_OPCODE_FORCE_DOCK);
}


// *****************************************************************************
// Compose a song
int iRobot_OI::setSong(unsigned char songNumber, unsigned char songLength, unsigned char *notes, unsigned char *noteLengths)
{
	int size = 2*songLength+3;
	unsigned char cmdBuffer[size];
	unsigned char i;
	
	cmdBuffer[0] = OI_OPCODE_SONG;
	cmdBuffer[1] = songNumber;
	cmdBuffer[2] = songLength;
	
	for(i=0 ; i < songLength ; i++)
	{
		cmdBuffer[3+(2*i)] = notes[i];
		cmdBuffer[3+(2*i)+1] = noteLengths[i];
	}
	
	if(write(fileDescriptor, cmdBuffer, size) < 0)
	{
		// Error
		return(-1);
	}
	else return(0);
}


// *****************************************************************************
// Play a song from the list
int iRobot_OI::playSong(unsigned char songNumber)
{
	unsigned char cmdBuffer[2];
	
	cmdBuffer[0] = OI_OPCODE_PLAY;
	//cmdBuffer[1] = songNumber;
	cmdBuffer[1] = 0;
	
	if(write(fileDescriptor, cmdBuffer, 2) < 0)
	{
		// Error
		return(-1);
	}
	else return(0);
}


// *****************************************************************************
// Set the LEDs
int iRobot_OI::setLeds(unsigned char checkRobot, unsigned char dock, unsigned char spot, unsigned char debris, unsigned char powerColor, unsigned char powerIntensity)
{
	unsigned char cmdBuffer[5];
	cmdBuffer[0] = OI_OPCODE_LEDS;
	cmdBuffer[1] = debris | spot<<1 | dock<<2 | checkRobot<<3;
	cmdBuffer[2] = powerColor;
	cmdBuffer[3] = powerIntensity;
	
	if(write(fileDescriptor, cmdBuffer, 4) < 0)
	{
		// Error
		return(-1);
	}
	else return(0);
}


// *****************************************************************************
// Set the scheduling LEDs
int iRobot_OI::setSchedulingLeds(unsigned char sun, unsigned char mon, unsigned char tue, unsigned char wed, unsigned char thu, unsigned char fri, unsigned char sat, unsigned char colon, unsigned char pm, unsigned char am, unsigned char clock, unsigned char schedule)
{
	// TODO: Not yet implemented...
	return(-1);
}


// *****************************************************************************
// Set the digit LEDs
int iRobot_OI::setDigitLeds(unsigned char digit3, unsigned char digit2, unsigned char digit1, unsigned char digit0)
{
	// TODO: Not yet implemented...
	return(-1);
}


// EOF
