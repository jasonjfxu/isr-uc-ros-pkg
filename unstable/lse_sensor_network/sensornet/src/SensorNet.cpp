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
* Author: Gonçalo Cabrita and Pedro Sousa on 25/10/2010
*********************************************************************/
#include "SensorNet.h"

SensorNet::SensorNet(std::string port_name, int baud_rate)
{
	serial_port = new cereal::CerealPort();
	
	try{ serial_port->open(port_name.c_str(), baud_rate); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("SensorNet -- Could not connect to the network!");
		ROS_BREAK();
	}
	
	start_time = ros::Time::now();
	
	timeout = 200; //ms
}

SensorNet::~SensorNet()
{
	try{ serial_port->close(); }
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet -- Could not close the serial port!");
	}

	delete serial_port;
}

bool SensorNet::sendCommand(char address, char command)
{
	std::string data;
	
	data+=SN_START;
	data+=address;
	data+=command;
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Sending command %c to address %c - %s", command, address, data.c_str());
	
	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - sendCommand - Error sending command %c to node %c.", command, address);
		return false;
	}
	return true;
}

bool SensorNet::waitForIt(char address)
{
	std::string reply;

	try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("SensorNet - waitForIt - Timeout waiting for ack from node %c.", address);
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet - waitForIt - Error reading ack from node %c: \n%s", address, e.what());
		return false;
	}
	
	if(reply[1]==address) return true;
	ROS_ERROR("SensorNet - waitForIt - Wrong ack received: got %c instead of %c", reply[1], address);
	return false;
}

bool SensorNet::scanNodes(int period)
{
	std::string reply;
	bool timedout;
	
	for(char i='A' ; i<SN_BROADCAST ; i++)
	{
		timedout=false;
		if(sendCommand(i, SN_GET_INFO))
		{
			try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
			catch(cereal::TimeoutException& te)
			{ 
				if(DEBUG) ROS_WARN("SensorNet - scanNodes - Timeout, skipping node %c.", i);
				timedout=true;
			}
			catch(cereal::Exception& e)
			{ 
				ROS_ERROR("SensorNet - scanNodes - Error reading from node %c: \n%s", i, e.what());
				return false;
			}
			
			if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Scanning... Reply from node %c - %s", i, reply.c_str());
			
			if(!timedout)
			{
				int version = extractInt(reply.substr(4,reply.size()-4));
				if(version==SN_VERSION)
				{
					nodes.push_back(SensorNet::Node(reply[1], reply[2], period));
					ROS_INFO("SensorNet - Found %s node on address %c with software version %d.", reply[2]==SN_SENSOR ? "a sensor" : "an actuator", i, SN_VERSION);
				}
				else
				{
					ROS_WARN("SensorNet - Found node %c but its software version %d is not compatible with version %d. Discarding this node->..", i, version, SN_VERSION);
				}
			}
		}
	}
	return true;
}

bool SensorNet::syncNodes()
{
	std::string data;

	data+=SN_START;
	data+=SN_BROADCAST;
	data+=SN_SYNC_TIME;
	appendTime(&data, ros::Time::now());
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Syncing all nodes - %s", data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - syncTime - Failed to sync nodes.");
		return false;
	}
	return true;
}

void SensorNet::appendTime(std::string * data, ros::Time time)
{
	ros::Duration t = time - start_time;

	int hh = 0;
	int mm = 0;
	int ss = (int)t.toSec();
	int d = (int)((t.toSec()-(double)ss)*10);
	
	if(ss>60)
	{
		mm = (int)(ss/60);
		ss -= 60*mm;
	}
	if(mm>60)
	{
		hh = (int)(mm/60);
		mm -= 60*hh;
	}
	if(hh>99)
	{
		hh=99;
		mm=99;
		ss=99;
		d=9;
	}
	
	char time_chars[8];
	sprintf(time_chars, "%02d%02d%02d%1d", hh, mm, ss, d);
	data->append(time_chars);
}

ros::Time SensorNet::extractTime(std::string data)
{
	char time_chars[3] = {0,0,0};
	double secs = 0.0;
	int temp;
	
	//hh
	time_chars[0] = data.at(0);
	time_chars[1] = data.at(1);
	sscanf(time_chars, "%d", &temp);
	secs = temp*60*60;
	//mm
	time_chars[0] = data.at(2);
	time_chars[1] = data.at(3);
	sscanf(time_chars, "%d", &temp);
	secs += temp*60;
	//ss
	time_chars[0] = data.at(4);
	time_chars[1] = data.at(5);
	sscanf(time_chars, "%d", &temp);
	secs += temp;
	//dd
	time_chars[0] = data.at(6);
	time_chars[1] = 0;
	sscanf(time_chars, "%d", &temp);
	secs += temp/10;
	
	return start_time + ros::Duration(secs);
}

void SensorNet::appendInt(std::string * data, int value)
{
	char value_str[8];
	sprintf(value_str, "%d", value);
	
	data->append(value_str);
}

int SensorNet::extractInt(std::string data)
{
	int value;

	sscanf(data.c_str(), "%d", &value);
	
	return value;
}

// Node functions
bool SensorNet::syncTime(SensorNet::Node * node)
{
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SYNC_TIME;
	appendTime(&data, ros::Time::now());
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Syncing time on node %c - %s", node->address(), data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - syncTime - Failed to sync node %c.", node->address());
		return false;
	}
	return waitForIt(node->address());
}

bool SensorNet::setTime(SensorNet::Node * node, ros::Time time)
{
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SET_TIME;
	appendTime(&data, time);
	data+=SN_SEP;
	appendInt(&data, node->period());
	data+=CR;

	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Setting time on node %c - %s", node->address(), data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - setTime - Failed to set time on node %c.", node->address());
		return false;
	}
	return waitForIt(node->address());
}

bool SensorNet::getTime(SensorNet::Node * node, ros::Time * current_time, ros::Time * next_reading)
{
	std::string reply;
	
	if(!sendCommand(node->address(), SN_GET_TIME)) return false;
	
	try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("SensorNet - getTime - Timeout getting time on node %c.", node->address());
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet - getTime - Error getting time from node %c: \n%s", node->address(), e.what());
		return false;
	}
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Getting time from node %c - %s", node->address(), reply.c_str());
	
	*current_time = extractTime(reply.substr(reply.find_first_of(node->address())+1,7));
	*next_reading = extractTime(reply.substr(reply.find_first_of(SN_SEP)+1,7));
	node->setPeriod(extractInt(reply.substr(reply.find_last_of(SN_SEP)+1,reply.size()-reply.find_last_of(SN_SEP)+1)));
	
	return true;
}

bool SensorNet::getSensorReadings(SensorNet::Node * node, std::vector<lse_sensor_msgs::Nostril> * mox, std::vector<lse_sensor_msgs::Anemometer> * thermistor)
{
	if(!node->sensor()) return false;
	
	std::string reply;
	
	if(!sendCommand(node->address(), SN_READ_SENSORS)) return false;
	
	try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("SensorNet - getSensorReadings - Timeout getting sensor readings header on node %c.", node->address());
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet - getSensorReadings - Error reading sensors header from node %c: \n%s", node->address(), e.what());
		return false;
	}
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Getting sensors from node %c - %s", node->address(), reply.c_str());
	
	ros::Time first_reading = extractTime(reply.substr(reply.find_first_of(node->address())+1,7));
	
	int n = extractInt(reply.substr(reply.find_last_of(SN_SEP)+1,reply.size()-reply.find_last_of(SN_SEP)+1));
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - %d sensor readings coming this way!!!", n);
	
	for(int i=0 ; i<n ; i++)
	{
		try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
		catch(cereal::TimeoutException& te)
		{ 
			ROS_ERROR("SensorNet - getSensorReadings - Timeout getting sensor readings on node %c.", node->address());
			return false;
		}
		catch(cereal::Exception& e)
		{ 
			ROS_ERROR("SensorNet - getSensorReadings - Error reading sensors from node %c: \n%s", node->address(), e.what());
			return false;
		}
		
		if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Getting sensor readings from node %c - %s", node->address(), reply.c_str());
	
		int sep = reply.find_first_of(SN_SEP);
		int next_sep = reply.find_first_of(SN_SEP,sep+1);
		if(next_sep==-1) next_sep = reply.size()-1;
		for(int j=0 ; j<4 ; j++)
		{
			if(node->sensors(j))
			{
				lse_sensor_msgs::Nostril moxMsg;
			
				char frame_id[32];
				sprintf(frame_id, "/base_node_%c_S%d", node->address(), j+1);
				moxMsg.header.frame_id = frame_id;
				moxMsg.header.stamp = first_reading + ros::Duration(node->period()*i);
				moxMsg.sensor_model = "Figaro 2620";
				moxMsg.gas_type.push_back(lse_sensor_msgs::Nostril::ORGANIC_SOLVENTS);
				moxMsg.reading = extractInt(reply.substr(sep+1,next_sep-sep));
				moxMsg.min_reading = 0.0;
				moxMsg.max_reading = 3300.0;
			
				if(mox!=NULL) mox->push_back(moxMsg);
				
				sep=next_sep;
				next_sep=reply.find_first_of(SN_SEP,sep+1);
				if(next_sep==-1) next_sep = reply.size()-1;
			}
		}
		for(int j=0 ; j<2 ; j++)
		{
			if(node->sensors(j+4))
			{
				lse_sensor_msgs::Anemometer termMsg;
				
				char frame_id[32];
				sprintf(frame_id, "/base_node_%c_W%d", node->address(), j+1);
				termMsg.header.frame_id = frame_id;
				termMsg.header.stamp = first_reading + ros::Duration(node->period()/10*i);
				//TODO: Finish the wind speed calculation...
				termMsg.wind_speed = extractInt(reply.substr(sep+1,next_sep-sep));
				termMsg.wind_direction = 0.0;
				
				if(thermistor!=NULL) thermistor->push_back(termMsg);
				
				sep=next_sep;
				next_sep=reply.find_first_of(SN_SEP,sep+1);
				if(next_sep==-1) next_sep = reply.size()-1;
			}
		}
		
	}
	return true;
}

bool SensorNet::setAllPWM(SensorNet::Node * node, int pwm1, int pwm2, int pwm3, int pwm4)
{
	if(!node->actuator()) return false;
	
	if(pwm1>100) pwm1=100;
	if(pwm1<0) pwm1=0;
	if(pwm2>100) pwm2=100;
	if(pwm2<0) pwm2=0;
	if(pwm3>100) pwm3=100;
	if(pwm3<0) pwm3=0;
	if(pwm4>100) pwm4=100;
	if(pwm4<0) pwm4=0;
	
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SET_ALL_PWM;
	appendInt(&data, pwm1);
	data+=SN_SEP;
	appendInt(&data, pwm2);
	data+=SN_SEP;
	appendInt(&data, pwm3);
	data+=SN_SEP;
	appendInt(&data, pwm4);
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Setting all PWMs on node %c - %s", node->address(), data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - setAllPWM - Failed to set pwm on node %c.", node->address());
		return false;
	}
	return waitForIt(node->address());
}

bool SensorNet::setPWM(SensorNet::Node * node, ros::Time time, unsigned int id, int pwm)
{
	if(!node->actuator()) return false;
	
	if(pwm>100) pwm=100;
	if(pwm<0) pwm=0;
	
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SET_ALL_PWM;
	appendTime(&data, time);
	data+=SN_SEP;
	appendInt(&data, id);
	data+=SN_SEP;
	appendInt(&data, pwm);
	data+=CR;

	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Setting PWM on node %c - %s", node->address(), data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - setPWM - Failed to set pwm on node %c.", node->address());
		return false;
	}
	return waitForIt(node->address());
}


bool SensorNet::getInfo(SensorNet::Node * node)
{
	std::string reply;
	
	char version[8];
	sprintf(version, "%d", SN_VERSION);
	
	if(!sendCommand(node->address(), SN_GET_INFO)) return false;
	
	try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("SensorNet - getInfo - Timeout getting info on node %c.", node->address());
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet - getInfo - Error getting info from node %c: \n%s", node->address(), e.what());
		return false;
	}
	
	if(node->address()!=reply[1])
	{
		ROS_ERROR("SensorNet - getInfo - Node %c came back with a different address: %c!", node->address(), reply[1]);
		return false;
	}
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Getting info on node %c - %s", node->address(), reply.c_str());
	
	node->setType(reply[2]);
	
	if(reply.compare(reply.size()-3,3,version)==0)
	{	
		ROS_WARN("SensorNet - getInfo - Node %c seems to have an incompatible version of the software.", node->address());
	}
	
	return true;
}

bool SensorNet::setSensors(SensorNet::Node * node, int s1, int s2, int s3, int s4, int w1, int w2)
{
	if(!node->sensor()) return false;
	
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SET_SENSORS;
	appendInt(&data, s1==0 ? 0 : 1);
	appendInt(&data, s2==0 ? 0 : 1);
	appendInt(&data, s3==0 ? 0 : 1);
	appendInt(&data, s4==0 ? 0 : 1);
	appendInt(&data, w1==0 ? 0 : 1);
	appendInt(&data, w2==0 ? 0 : 1);
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Setting sensors on node %c - %s", node->address(), data.c_str());

	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - setSensors - Failed to set sensors on node %c.", node->address());
		return false;
	}
	
	if(!waitForIt(node->address())) return false;
	
	node->setSensors(s1,s2,s3,s4,w1,w2);
	return true;
}

bool SensorNet::getSensors(SensorNet::Node * node)
{
	if(!node->sensor()) return false;

	std::string reply;
	
	if(!sendCommand(node->address(), SN_GET_SENSORS)) return false;
	
	try{ serial_port->readBetween(&reply, SN_START, (char)CR, timeout); }
	catch(cereal::TimeoutException& te)
	{ 
		ROS_ERROR("SensorNet - getSensors - Timeout getting sensors on node %c.", node->address());
		return false;
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("SensorNet - getSensors - Error getting sensors from node %c: \n%s", node->address(), e.what());
		return false;
	}
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Getting sensors from node %c - %s", node->address(), reply.c_str());
	
	node->setSensors((int)(reply[2]-48),(int)(reply[3]-48),(int)(reply[4]-48),(int)(reply[5]-48),(int)(reply[6]-48),(int)(reply[7]-48));
	
	return true;
}

bool SensorNet::setAddress(SensorNet::Node * node, char new_address)
{
	std::string data;

	data+=SN_START;
	data+=node->address();
	data+=SN_SET_ADDRESS;
	data+=new_address;
	data+=CR;
	
	if(DEBUG) ROS_INFO("SensorNet [DEBUG] - Setting a new address %c on node %c - %s", new_address, node->address(), data.c_str());
	
	try{ serial_port->write(data.c_str()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("SensorNet - setAddress - Failed to set a new address %c on node %c.", new_address, node->address());
		return false;
	}

	if(!waitForIt(new_address)) return false;
	node->setAddress(new_address);
	return true;			
}

// EOF
