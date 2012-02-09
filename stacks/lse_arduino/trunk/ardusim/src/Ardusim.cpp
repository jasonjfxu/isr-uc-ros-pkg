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

#include "ardusim/Ardusim.h"

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
		
			nose.raw_data = getValue(data);
		
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
			
			int ch[4];
			for(int j=0 ; j<4 ; j++)
			{
				ch[j] = getValue(data);
				raw.reading = ch[j];
				thermistor_msgs_.push_back(raw);
			}
		
			// And now_ the anemometer msg
			lse_sensor_msgs::Anemometer anemometer;
			anemometer.header.stamp = now_;
			
			if(calib_data_.size()>0)
			{
				std::vector<TopDistances> candidates;
			
				float top_distance_sums[2] = {-1, -1};
				int ind[2];
			
				for(int k=0 ; k<calib_data_.size() ; k++)
				{
					float ch0 = ch[0]-calib_data_[k].ch0;
					float ch1 = ch[1]-calib_data_[k].ch1;
					float ch2 = ch[2]-calib_data_[k].ch2;
					float ch3 = ch[3]-calib_data_[k].ch3;
					float distance = ch0*ch0 + ch1*ch1 + ch2*ch2 + ch3*ch3;
				
					int candidate_ind = -1;
					for(int c=0 ; c<candidates.size() ; c++)
					{
						if(candidates[c].velocity == calib_data_[k].velocity)
						{
							candidate_ind = c;
							break;
						}
					}
					
					if(candidate_ind < 0)
					{
						TopDistances new_candidate;
						new_candidate.velocity = calib_data_[k].velocity;
						new_candidate.index[0] = k;
						new_candidate.distance[0] = distance;
						new_candidate.index[1] = -1;
						new_candidate.distance[1] = 0.0;
						candidates.push_back(new_candidate);
					}
					else
					{
						if(distance < candidates[candidate_ind].distance[0])
						{
							candidates[candidate_ind].distance[1] = candidates[candidate_ind].distance[0];
							candidates[candidate_ind].index[1] = candidates[candidate_ind].index[0];
							candidates[candidate_ind].distance[0] = distance;
							candidates[candidate_ind].index[0] = k;
						}
						else if(distance < candidates[candidate_ind].distance[1] || candidates[candidate_ind].index[1] == -1)
						{
							candidates[candidate_ind].distance[1] = distance;
							candidates[candidate_ind].index[1] = k;
						}
					
						float distance_sum = candidates[candidate_ind].distance[0]+candidates[candidate_ind].distance[1];
					
						if(distance_sum < top_distance_sums[0] || top_distance_sums[0] == -1)
						{
							top_distance_sums[1] = top_distance_sums[0];
							ind[1] = ind[0];
							top_distance_sums[0] = distance_sum;
							ind[0] = candidate_ind;
						}
						else if(distance_sum < top_distance_sums[1] || top_distance_sums[1] == -1)
						{
							top_distance_sums[1] = distance_sum;
							ind[1] = candidate_ind;
						}
					}

				}
			
				float mean_weight = (top_distance_sums[0] + top_distance_sums[1]) / 4.0;
				candidates[ind[0]].weight[0] = candidates[ind[0]].distance[0]==0 ? mean_weight/0.01 : mean_weight/candidates[ind[0]].distance[0];
				candidates[ind[0]].weight[1] = candidates[ind[0]].distance[1]==0 ? mean_weight/0.01 : mean_weight/candidates[ind[0]].distance[1];
				candidates[ind[1]].weight[0] = candidates[ind[1]].distance[0]==0 ? mean_weight/0.01 : mean_weight/candidates[ind[1]].distance[0];
				candidates[ind[1]].weight[1] = candidates[ind[1]].distance[1]==0 ? mean_weight/0.01 : mean_weight/candidates[ind[1]].distance[1];
			
				float sum = candidates[(int)*ind].weight[0] + candidates[(int)*ind].weight[1] + candidates[(int)*ind+1].weight[0] +candidates[(int)*ind+1].weight[1];
				candidates[ind[0]].weight[0] /= sum;
				candidates[ind[0]].weight[1] /= sum;
				candidates[ind[1]].weight[0] /= sum;
				candidates[ind[1]].weight[1] /= sum;
			
				anemometer.wind_direction = calib_data_[candidates[ind[0]].index[0]].angle*candidates[ind[0]].weight[0] + calib_data_[candidates[ind[0]].index[1]].angle*candidates[ind[0]].weight[1] + calib_data_[candidates[ind[1]].index[0]].angle*candidates[ind[1]].weight[0] + calib_data_[candidates[ind[1]].index[1]].angle*candidates[ind[1]].weight[1];
				anemometer.wind_speed = calib_data_[candidates[ind[0]].index[0]].velocity*candidates[ind[0]].weight[0] + calib_data_[candidates[ind[0]].index[1]].velocity*candidates[ind[0]].weight[1] + calib_data_[candidates[ind[1]].index[0]].velocity*candidates[ind[1]].weight[0] + calib_data_[candidates[ind[1]].index[1]].velocity*candidates[ind[1]].weight[1];
		
				anemometer_msgs_.push_back(anemometer);
			}
		}
	}
	
	bool Ardusim::loadAnemometerCalibFile(std::string * file_path)
	{
		FILE * calib_file;
		
		calib_file = fopen(file_path->c_str(), "r");
		
		if(calib_file==NULL) return false;
		
		while(!feof(calib_file))
		{
			AnemometerCalibData new_calib_data;
			
			fscanf(calib_file, "%f,%d,%d,%d,%d,%f\n", &new_calib_data.angle, &new_calib_data.ch0, &new_calib_data.ch1, &new_calib_data.ch2, &new_calib_data.ch3, &new_calib_data.velocity);
			
			//ROS_INFO("%f,%d,%d,%d,%d,%f", new_calib_data.angle, new_calib_data.ch0, new_calib_data.ch1, new_calib_data.ch2, new_calib_data.ch3, new_calib_data.velocity);
			
			calib_data_.push_back(new_calib_data);
		}
		
		return true;
	}

} // namespace

// EOF

