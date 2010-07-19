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
 * \file wifi_comm_node.cpp
 *
 * \brief This file implements the multi robot communication.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 12/2010
 * \version 0.1
 *
 * \bug none discovered
 *
 * \note This file is incomplete since this idea was not pursued.
 *
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include "wifi_comm/WiFiNeighboursList.h"
#include "wifi_comm/WiFiCommContainer.h"

// defines
#define NODE_VERSION 0.01


typedef struct
{
	int pid;
	std::string ip;

} ForeignRelay;

std::vector<ForeignRelay> ForeignRelays;


void openForeignRelay(char * ip)
{
	ForeignRelay newRelay;

	int pid = fork();
	if(pid==0)	// Child
	{
		char ros_master_uri[128];
		sprintf(ros_master_uri, "http://%s:11311", ip);

		execlp("rosrun", "rosrun", "foreign_relay", "foreign_relay", "adv", ros_master_uri, "/wifi_comm_rcv", "/wifi_comm_snd", (char*)0);
		
		ROS_WARN("Reached the end of foreign_relay to %s with pid %d", ip, getpid());
		exit(0);
	}
	else if(pid<0)	// Error
	{
		ROS_ERROR("Failed to open foreign_relay to %s", ip);
	}
	else		// Parent
	{
		newRelay.pid = pid;
		newRelay.ip = ip;
		ForeignRelays.push_back(newRelay);
		ROS_INFO("Successfully opened foreign_relay to %s with pid %d", ip, pid);
	}
	return;
}

void closeForeignRelay(char * ip)
{
	unsigned int i;

	for(i=0 ; i<ForeignRelays.size() ; i++)
	{
		if(ForeignRelays[i].ip.compare(ip)==0)
		{
			ROS_INFO("Killing foreign_relay to %s with pid %d", ip, ForeignRelays[i].pid);
			kill(ForeignRelays[i].pid, SIGTERM);
			ForeignRelays.erase(ForeignRelays.begin()+i);
			return;
		}
	}
	
	ROS_ERROR("Could not find foreign_relay to %s", ip);
	return;
}

void neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg)
{
	unsigned int i, j;
	bool found;

  	// Now open and close foreign_relays!
	for(i=0 ; i<msg->neighbours.size() ; i++)
	{
		found = false;
		for(j=0 ; j<ForeignRelays.size() ; j++)
		{
			if(msg->neighbours[i].ip.compare(ForeignRelays[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		if(!found) openForeignRelay((char*)msg->neighbours[i].ip.c_str());
	}

	for(i=0 ; i<ForeignRelays.size() ; i++)
	{
		found = false;
		for(j=0 ; j<msg->neighbours.size() ; j++)
		{
			if(ForeignRelays[i].ip.compare(msg->neighbours[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		if(!found) closeForeignRelay((char*)ForeignRelays[i].ip.c_str());
	}
}


// *****************************************************************************
// Main function for the multiRobotCom node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "wifi_comm");
	ros::NodeHandle n;
	
	ROS_INFO("Wifi Comm Node for ROS %.2f", NODE_VERSION);

	ros::Publisher pub = n.advertise<wifi_comm::WiFiCommContainer>("/wifi_comm_snd", 10);
	ros::Subscriber list_sub = n.subscribe("wifi_neighbours_list", 10, neighboursListUpdated);
 	ros::spin();

	return 0;
}

// EOF

