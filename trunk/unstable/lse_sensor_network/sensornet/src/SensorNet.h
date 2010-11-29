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
#define NODE_VERSION 0.01

#include <ros/ros.h>
#include <lse_sensor_msgs/Nostril.h>
#include <lse_sensor_msgs/Anemometer.h>

#include <vector>
#include <string>

#include "cereal_port/CerealPort.h"

#define DEBUG 			0

#define CR	13
#define SN_START		':'
#define SN_SEP			','

#define SN_BROADCAST 	'Z'

#define SN_SYNC_TIME 	'a'
#define SN_SET_TIME		'b'
#define SN_READ_SENSORS	'c' 	
#define SN_GET_POWER	'd'
#define SN_GET_TIME		'e'
#define SN_SET_ALL_PWM	'f'
#define SN_SET_PWM		'g'
#define SN_SET_ADDRESS	'h'
#define SN_SET_SENSORS	'i'
#define SN_GET_SENSORS	'j'
#define SN_GET_INFO		'x'

#define SN_SENSOR		's'
#define SN_ACTUATOR		'a'

#define SN_VERSION 		10

class SensorNet
{
	public:
	SensorNet(std::string port_name, int baud_rate);
	~SensorNet();
	
	class Node
	{
		public:
		Node(){}
		Node(char address, char type, int period)
		{
			this->node_address=address;
			this->node_type=type;
			this->node_period=period;
		}
		~Node(){}
		
		char address(){return node_address;}
		char type(){return node_type;}
		int period(){return node_period;}
		bool sensors(int i){return node_sensors[i];}
		
		void setAddress(char address){node_address=address;}
		void setType(char type){node_type=type;}
		void setPeriod(int period){node_period=period;}
		void setSensors(int s1, int s2, int s3, int s4, int w1, int w2)
		{
			node_sensors[0] = s1==0 ? false : true;
			node_sensors[1] = s2==0 ? false : true;
			node_sensors[2] = s3==0 ? false : true;
			node_sensors[3] = s4==0 ? false : true;
			node_sensors[4] = w1==0 ? false : true;
			node_sensors[5] = w2==0 ? false : true;
		}
		
		bool sensor(){return node_type==SN_SENSOR;}
		bool actuator(){return node_type==SN_ACTUATOR;}
		
		private:
		char node_address;
		char node_type;
		int node_period;		//dd
		
		bool node_sensors[6];	//S1 S2 S3 S4 W1 W2
	};
	
	std::vector<Node> nodes;
	
	bool scanNodes(int period);
	bool syncNodes();
	
	bool syncTime(SensorNet::Node * node);
	bool setTime(SensorNet::Node * node, ros::Time start_time);
	bool getTime(SensorNet::Node * node, ros::Time * current_time, ros::Time * next_reading);
	bool getSensorReadings(SensorNet::Node * node, std::vector<lse_sensor_msgs::Nostril> * mox, std::vector<lse_sensor_msgs::Anemometer> * termistor);
	bool setAllPWM(SensorNet::Node * node, int pwm1, int pwm2, int pwm3, int pwm4);
	bool setPWM(SensorNet::Node * node, ros::Time time, unsigned int id, int pwm);
	bool getInfo(SensorNet::Node * node);
	bool setSensors(SensorNet::Node * node, int s1, int s2, int s3, int s4, int w1, int w2);
	bool getSensors(SensorNet::Node * node);
	bool setAddress(SensorNet::Node * node, char new_address);
	
	int timeout;
	
	private:
	cereal::CerealPort * serial_port;
	
	ros::Time start_time;
	
	bool sendCommand(char address, char command);
	bool waitForIt(char address);
	
	void appendTime(std::string * data, ros::Time time);
	void appendInt(std::string * data, int value);
	ros::Time extractTime(std::string data);
	int extractInt(std::string data);
};

// EOF
