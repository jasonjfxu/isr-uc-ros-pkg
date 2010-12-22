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
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>

#include <list>
#include <vector>
#include <string>

#include <lse_sensor_msgs/Thermistor.h>
#include <lse_sensor_msgs/Anemometer.h>
#include <lse_sensor_msgs/Nostril.h>
#include <lse_sensor_msgs/TPA.h>
#include <lse_sensor_msgs/Range.h>

#define VERSION 			10

// Sensor modules
#define ARDUSIM_RANGE 		0
#define ARDUSIM_NOSE		1	
#define ARDUSIM_TPA			2
#define ARDUSIM_ANEMOMETER	3
#define ARDUSIM_NUNCHUCK	4
#define ARDUSIM_SHTXX		5

// Modes
#define ARDUSIM_QUERY    'Q'
#define ARDUSIM_STREAM   'S'
#define ARDUSIM_INFO	 'I'
#define ARDUSIM_AUTO	 'A'
#define ARDUSIM_VERSION	 'V'

// Chars
#define STRT	'@'
#define END		'e'
#define SPRT	','

namespace ardusim
{

/*! \class Ardusim Ardusim.h "inc/Ardusim.h"
 *  \brief C++ class implementation of the ardusim protocol.
 *
 * This class implements the ardusim protocol (arduino sensor interface module). This allows for easy ROS publishing of LSE common sensor data. 
 */
class Ardusim
{
	public:
		
		//! Constructor
		/*!
		 *
		 *  \param port_name    Name of the serial port to open.
		 * 
		 */
		Ardusim(std::string port_name);
		//! Destructor
		~Ardusim();
		
		//! Check version
		/*!
		*  Check the software version running in the arduino for compatibility issues.
		*
		*  \param timeout  Timeout in milliseconds.
		* 
		*  \return The version of the sofware in the arduino.
		*/
		int checkVersion(int timeout);

		//! Scan i2c sensors.
		/*!
		*  Commands the arduino to perform a i2c scan to detect connected sensors automatically. Wrapper for autoRequests.
		*
		*  \param timeout  Timeout in milliseconds.
		*
		*  \sa autoRequests()
		*  \sa setRequests()
		* 
		*  \return True if succeeded, false if not.
		*/
		bool sensorDiscovery(int timeout){ return autoRequests(timeout, ARDUSIM_AUTO); }
		//! Ask for connected sensors.
		/*!
		*  Asks the arduino which user-defined sensors are connected. Wrapper for autoRequests.
		*
		*  \param timeout  Timeout in milliseconds.
		*
		*  \sa autoRequests()
		*  \sa setRequests()
		* 
		*  \return True if succeeded, false if not.
		*/
		bool setAutoRequests(int timeout){ return autoRequests(timeout, ARDUSIM_INFO); }
		
		//! Set requests
		/*!
		*  Tell the arduino which sensors to read.
		*
		*  \param requests  Array of requests to read.
		*  \param num_of_requests  Number of requests.
		*
		*/
		void setRequests(int * requests, int num_of_requests);
		//! Get requests
		/*!
		*  Ask Ardusim which sensors we are getting!
		*
		*  \param requests  List of requests.
		*
		*/
		void getRequests(std::list<int> * requests);
		//! Read sensor data from the arduino
		/*!
		*  Read sensor data from the arduino. The sensors requested are the ones in the list of requests.
		*
		*  \param timeout  Timeout in milliseconds.
		*
		*  \sa requests_
		*  \sa sensorDiscovery()
		*  \sa setAutoRequests()
		*  \sa setRequests()
		* 
		*  \return True if succeeded, false if not.
		*/
		bool getSensorData(int timeout);
	
		//! Get value
		/*!
		 *  Load the calibration data for the anemometer from a file.
		 *
		 *  \param file_path  	File path.
		 *
		 *  \sa parseAnemometer()
		 *
		 *  \return Wether the file was loaded or not.
		 */
		bool loadAnemometerCalibFile(std::string * file_path);
		
		// Getters
		bool getRange(std::vector<lse_sensor_msgs::Range> * range);
		bool getNose(std::vector<lse_sensor_msgs::Nostril> * nose);
		bool getTPA(std::vector<lse_sensor_msgs::TPA> * tpa);
		bool getAnemometer(std::vector<lse_sensor_msgs::Anemometer> * anemometer);
		bool getThermistor(std::vector<lse_sensor_msgs::Thermistor> * thermistor);
		
	private:
	
		// Serial Port
		cereal::CerealPort * serial_port_;
		
		//! List of requests
		std::list<int> requests_;
		//! Current time
		ros::Time now_;
		
		//! Send command
		/*!
		*  Send a command to the arduino.
		*
		*  \param timeout  	Timeout in milliseconds.
		*  \param mode  	Ardusim mode.
		*  \param reply  	Reply from the arduino.
		* 
		*  \return True if succeeded, false if not.
		*/
		bool sendCommand(int timeout, char mode, std::string * reply);
		
		//! Auto request sensors from the arduino
		/*!
		*  Ask the arduino which sensors are connected to it. This can be user-defined or product of a i2c scan.
		*
		*  \param timeout  	Timeout in milliseconds.
		*  \param mode  	Ardusim mode.
		*
		*  \sa sensorDiscovery()
		*  \sa setAutoRequests()
		* 
		*  \return True if succeeded, false if not.
		*/
		bool autoRequests(int timeout, char mode);
		
		//! Add value
		/*!
		*  Parsing helper function. Adds an int value to a std::string. 
		*
		*  \param data  	String.
		*  \param value  	Int value to be added.
		*
		*  \sa getValue()
		* 
		*/
		void addValue(std::string * data, int value);
		//! Get value
		/*!
		*  Parsing helper function. Gets an int value from a std::string. 
		*
		*  \param data  	String.
		*
		*  \sa addValue()
		*
		*  \return An int value.
		*/
		int getValue(std::string * data);
		
		
		//! Parse data
		/*!
		*  Data parsing function. Parses data comming from the arduino.
		*
		*  \param data  	String to be parsed.
		*
		*  \return True if succeeded, false if not.
		*/
		bool parseData(std::string * data);
		
		//! Range data parser
		void parseRange(std::string * data);
		//! Nose data parser
		void parseNose(std::string * data);
		//! TPA data parser
		void parseTPA(std::string * data);
		//! Anemometer data parser
		void parseAnemometer(std::string * data);
		
		//! Vector of range messages
		std::vector<lse_sensor_msgs::Range> range_msgs_;
		//! Vector of nose messages
		std::vector<lse_sensor_msgs::Nostril> nose_msgs_;
		//! Vector of tpa messages
		std::vector<lse_sensor_msgs::TPA> tpa_msgs_;
		//! Vector of anemometer messages
		std::vector<lse_sensor_msgs::Anemometer> anemometer_msgs_;
		//! Vector of thermistor messages
		std::vector<lse_sensor_msgs::Thermistor> thermistor_msgs_;
	
		struct AnemometerCalibData
		{
			float angle;
			float velocity;
			int ch0, ch1, ch2, ch3;
		};
	
		std::vector<AnemometerCalibData> calib_data_;
		
		struct TopDistances
		{
			float velocity;
			
			int index[2];
			float distance[2];
			float weight[2];
		};
};

} // namespace

// EOF

