/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 14/10/2010
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "Ardusim.h"

namespace ardusim
{

// *****************************************************************************
// Constructor
Ardusim::Ardusim(std::string port_name)
{	
	serial_port_ = new cereal::CerealPort();
	try{ serial_port_->open(port_name.c_str(), 57600); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("Ardusim -- Could not connect to the Arduino on port %s", port_name.c_str());
		ROS_BREAK();
	}
	
	ROS_INFO("Ardusim -- Opening a connection to an Arduino on port %s...", port_name.c_str());	
	usleep(1500000);
	ROS_INFO("Ardusim -- Connection opened!");
	
	int version = checkVersion(500);
	if(version==-1)
	{
		ROS_FATAL("Ardusim -- The Arduino is not loaded with the proper software!");
		ROS_BREAK();
	}
	else if(version!=VERSION)
	{
		ROS_FATAL("Ardusim -- The Arduino software version %d is not compatible with Ardusim for ROS version %d!", version, VERSION);
		ROS_BREAK();
	}
	
	return;
}

// *****************************************************************************
// Destructor
Ardusim::~Ardusim()
{
	try{ serial_port_->close(); }
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("Ardusim -- Could not close the serial port!");
	}

	delete serial_port_;
}

// *****************************************************************************
// Send a command to the Arduino.
bool Ardusim::sendCommand(int timeout, char mode, std::string * reply)
{
	std::string data(1, STRT);
	
	data += SPRT;
	data += mode;
	data += SPRT;
	data += END;
		
	try{ serial_port_->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("Ardusim - getSensorData - Error writing to the Arduino!");
		return false;
	}
	
	try{ serial_port_->readBetween(reply, STRT, END, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("Ardusim - getSensorData - Timeout reading from the Arduino!");
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("Ardusim - getSensorData - Error reading from the Arduino: \n%s", e.what());
		return false;
	}
	
	return true;
}

// *****************************************************************************
// Check if the version of the Arduino software is compatible with this lib.
int Ardusim::checkVersion(int timeout)
{
	std::string reply;
	
	if(!sendCommand(timeout, ARDUSIM_VERSION, &reply)) return -1;
	
	// Take @, out
	reply.erase(0,2);
	
	return getValue(&reply);
}

// *****************************************************************************
// Determine automatically which sensors are connected to the arduino
bool Ardusim::autoRequests(int timeout, char mode)
{
	std::string reply;
	
	if(!sendCommand(timeout, mode, &reply)) return false;
	
	// Take @, out
	reply.erase(0,2);
	
	int n = getValue(&reply);
	
	requests_.clear();
	for(int i=0 ; i<n ; i++)
	{
		int sensor = getValue(&reply);
		if(sensor==ARDUSIM_RANGE || sensor==ARDUSIM_NOSE || sensor==ARDUSIM_TPA || sensor==ARDUSIM_ANEMOMETER) requests_.push_back(sensor);
		else if(sensor==ARDUSIM_NUNCHUCK) ROS_WARN("Ardusim GPnode - Discarding Nunchuck: unsupported");
		else if(sensor==ARDUSIM_SHTXX) ROS_WARN("Ardusim GPnode - Discarding SHTxx: unsupported");
		else ROS_WARN("Ardusim GPnode - Discarding unknow_n sensor id %d", sensor);
	}
	requests_.sort();
	
	std::string end(1, END);
	if(reply.compare(end)==0) return true;
	else return false;
	return true;
}

// *****************************************************************************
// Get the current requests_ list
void Ardusim::getRequests(std::list<int> * requests_)
{
	*requests_ = this->requests_;
}

// *****************************************************************************
// Read data from the serial port
void Ardusim::setRequests(int * requests_, int num_of_requests_)
{
	this->requests_.clear();

	for(int i=0 ; i<num_of_requests_ ; i++) this->requests_.push_back(requests_[i]);
	
	this->requests_.sort();
}

// *****************************************************************************
// Read data from the serial port
bool Ardusim::getSensorData(int timeout)
{
	std::list<int>::iterator it;
	std::string reply;
	std::string data(1, STRT);
	
	data += SPRT;
	data += ARDUSIM_QUERY;
	data += SPRT;
	addValue(&data, requests_.size());
	for(it=requests_.begin() ; it!=requests_.end() ; ++it) addValue(&data, *it);
	data += END;
	
	try{ serial_port_->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("Ardusim - getSensorData - Error writing to the Arduino!");
		return false;
	}
	
	try{ serial_port_->readBetween(&reply, STRT, END, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("Ardusim - getSensorData - Timeout reading from the Arduino!");
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("Ardusim - getSensorData - Error reading from the Arduino: \n%s", e.what());
		return false;
	}
	
	now_ = ros::Time::now();
	
	return parseData(&reply);
}

// *****************************************************************************
// Range getter function
bool Ardusim::getRange(std::vector<lse_sensor_msgs::Range> * range)
{
	std::list<int>::iterator it;
	
	for(it=requests_.begin() ; it!=requests_.end() ; ++it)
	{
		if(*it==ARDUSIM_RANGE)
		{
			*range = range_msgs_;
			if(range_msgs_.size()==0) return false;
			return true;
		}
	}
	return false;
}

// *****************************************************************************
// Nose getter function
bool Ardusim::getNose(std::vector<lse_sensor_msgs::Nostril> * nose)
{
	std::list<int>::iterator it;
	
	for(it=requests_.begin() ; it!=requests_.end() ; ++it)
	{
		if(*it==ARDUSIM_NOSE)
		{
			*nose = nose_msgs_;
			if(nose_msgs_.size()==0) return false;
			return true;
		}
	}
	return false;
}

// *****************************************************************************
// TPA getter function
bool Ardusim::getTPA(std::vector<lse_sensor_msgs::TPA> * tpa)
{
	std::list<int>::iterator it;
	
	for(it=requests_.begin() ; it!=requests_.end() ; ++it)
	{
		if(*it==ARDUSIM_TPA)
		{
			*tpa = tpa_msgs_;
			if(tpa_msgs_.size()==0) return false;
			return true;
		}
	}
	return false;
}

// *****************************************************************************
// Anemometer getter function
bool Ardusim::getAnemometer(std::vector<lse_sensor_msgs::Anemometer> * anemometer)
{
	std::list<int>::iterator it;
	
	for(it=requests_.begin() ; it!=requests_.end() ; ++it)
	{
		if(*it==ARDUSIM_ANEMOMETER)
		{
			*anemometer = anemometer_msgs_;
			if(anemometer_msgs_.size()==0) return false;
			return true;
		}
	}
	return false;
}

// *****************************************************************************
// Anemometer getter function
bool Ardusim::getThermistor(std::vector<lse_sensor_msgs::Thermistor> * thermistor)
{
	std::list<int>::iterator it;
	
	for(it=requests_.begin() ; it!=requests_.end() ; ++it)
	{
		if(*it==ARDUSIM_ANEMOMETER)
		{
			*thermistor = thermistor_msgs_;
			if(thermistor_msgs_.size()==0) return false;
			return true;
		}
	}
	return false;
}


// *****************************************************************************
// Add an int value to the string
void Ardusim::addValue(std::string * data, int value)
{
	char value_str[8];
	sprintf(value_str, "%d", value);
	
	data->append(value_str);
	data->append(1, SPRT);
}

// *****************************************************************************
// Get an int value from the string
int Ardusim::getValue(std::string * data)
{
	int value;
	
	int pos = data->find_first_of(SPRT);
	std::string sub = data->substr(0,pos+1);
	data->erase(0,pos+1);
	
	if( sscanf(sub.c_str(), "%d,", &value) == 1) return value;
	else return -1;
}

// *****************************************************************************
// Parsing function
bool Ardusim::parseData(std::string * data)
{	
	char sensor;
	
	// Take @, out
	data->erase(0,2);
	
	std::string end(1, END);
	while(data->compare(end)!=0)
	{
		sensor = getValue(data);
		
		if(sensor==ARDUSIM_RANGE) parseRange(data);
		else if(sensor==ARDUSIM_NOSE) parseNose(data);
		else if(sensor==ARDUSIM_TPA) parseTPA(data);
		else if(sensor==ARDUSIM_ANEMOMETER) parseAnemometer(data);
		else
		{
			ROS_ERROR("Ardusim - parseData - Undefined sensor received!");
			return false;
		}
		if(data->size()==0)
		{
			ROS_ERROR("Ardusim - parseData - Reached the end of the buffer!");
			return false;
		}
	}
	return true;
}

// *****************************************************************************
// Parsing function
void Ardusim::parseRange(std::string * data)
{
	int n = getValue(data);
	
	range_msgs_.clear();
	for(int i=0 ; i<n ; i++)
	{
		lse_sensor_msgs::Range range;
		range.header.stamp = now_;
		
		range.range = (float)(getValue(data)/100.0);
		
		range_msgs_.push_back(range);
	}
}

// *****************************************************************************
// Parsing function
void Ardusim::parseNose(std::string * data)
{
	int n = getValue(data);
	
	nose_msgs_.clear();
	for(int i=0 ; i<n ; i++)
	{
		lse_sensor_msgs::Nostril nose;
		nose.header.stamp = now_;
		
		nose.reading = (float)getValue(data);
		
		nose_msgs_.push_back(nose);
	}
}


// *****************************************************************************
// Parsing function
void Ardusim::parseTPA(std::string * data)
{
	int n = getValue(data);
	
	tpa_msgs_.clear();
	for(int i=0 ; i<n ; i++)
	{
		lse_sensor_msgs::TPA tpa;
		tpa.header.stamp = now_;
		
		tpa.room_temperature = getValue(data);
		for(int j=0 ; j<8 ; j++) tpa.temperature.push_back(getValue(data)); 
		
		tpa_msgs_.push_back(tpa);
	}
}

// *****************************************************************************
// Parsing function
void Ardusim::parseAnemometer(std::string * data)
{
	int n = getValue(data);
	
	thermistor_msgs_.clear();
	anemometer_msgs_.clear();
	for(int i=0 ; i<n ; i++)
	{
		// The raw msg
		lse_sensor_msgs::Thermistor raw;
		raw.header.stamp = now_;
			
		for(int j=0 ; j<4 ; j++)
		{
			raw.reading = getValue(data)/8.0;
			thermistor_msgs_.push_back(raw);
		}
		
		// And now_ the anemometer msg
		lse_sensor_msgs::Anemometer anemometer;
		anemometer.header.stamp = now_;
		
		// TODO: Finish this!!!	
		anemometer.wind_speed = 0.0;
		anemometer.wind_direction = 0.0;
		
		anemometer_msgs_.push_back(anemometer);
	}
}

} // namespace

// EOF

