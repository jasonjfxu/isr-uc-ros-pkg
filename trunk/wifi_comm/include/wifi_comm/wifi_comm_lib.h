/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*!
 *
 * \file wifi_comm_lib.h
 *
 * \brief This file implements the multi robot communication library.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 15/2010
 * \version 0.1
 *
 * \bug none discovered
 *
 * \note
 *
 */

#ifndef _WIFI_COMM_H
#define _WIFI_COMM_H

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include "wifi_comm/WiFiNeighboursList.h"

#define STR_SIZE 128

typedef struct
{
	int pid;
	std::string ip;

} ForeignRelay;

class WiFiComm
{
	public:
	WiFiComm(void(*fp)(char * ip));
	~WiFiComm();
	
	static char * concatTopicAndIp(char * final_topic, char * topic, char * ip);

	std::string * openForeignRelay(char * ip, char * topic, bool broadcast);

	wifi_comm::WiFiNeighboursList neighboursList;
	
	private:
	ros::NodeHandle n;
	ros::Subscriber neighbours_sub;

	std::vector<ForeignRelay> ForeignRelays;

	void closeForeignRelay(char * ip);

	void neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg);
	
	void(*cbFunction)(char * ip);
};

#endif        /* _WIFI_COMM_H */

// EOF

