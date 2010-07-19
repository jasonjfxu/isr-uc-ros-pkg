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
 * \file wifi_comm_lib.cpp
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

#include "wifi_comm/wifi_comm_lib.h"

WiFiComm::WiFiComm(void(*fp)(char * ip)) : n()
{
	neighbours_sub = n.subscribe("wifi_neighbours_list", 10, &WiFiComm::neighboursListUpdated, this);

        cbFunction = fp;
}

WiFiComm::~WiFiComm()
{
	// Clean up
}

char * WiFiComm::concatTopicAndIp(char * final_topic, char * topic, char * ip)
{
	sprintf(final_topic, "%s_%s", topic, ip);

	for(int i=0 ; i<strlen(final_topic) ; i++)
	{
		if(final_topic[i]=='.') final_topic[i]='_'; 
	}
	return final_topic;
}

std::string * WiFiComm::openForeignRelay(char * ip, char * topic, bool broadcast)
{
	ForeignRelay newRelay;

	int pid = fork();
	if(pid==0)	// Child
	{
		char foreign_topic[STR_SIZE];
		char local_topic[STR_SIZE];
		
		if(broadcast) strcpy(local_topic, topic);
		else concatTopicAndIp(local_topic, topic, ip);

		char ros_master_uri[STR_SIZE];
		sprintf(ros_master_uri, "http://%s:11311", ip);
		execlp("rosrun", "rosrun", "foreign_relay", "foreign_relay", "adv", ros_master_uri, concatTopicAndIp(foreign_topic, topic, ip), local_topic, (char*)0);
		
		ROS_WARN("Reached the end of foreign_relay to %s on the topic %s with pid %d", ip, topic, getpid());
		exit(0);
	}
	else if(pid<0)	// Error
	{
		ROS_ERROR("Failed to open foreign_relay to %s on the topic %s", ip, topic);
		return NULL;
	}
	else		// Parent
	{
		newRelay.pid = pid;
		newRelay.ip = ip;
		ForeignRelays.push_back(newRelay);
		ROS_INFO("Successfully opened foreign_relay to %s on the topic %s with pid %d", ip, topic, pid);
		return &newRelay.ip;
	}
	return NULL;
}

void WiFiComm::closeForeignRelay(char * ip)
{
	unsigned int i;

	for(i=0 ; i<ForeignRelays.size() ; i++)
	{
		if(ForeignRelays[i].ip.compare(ip)==0)
		{
			// Apparently killing one is not enough! Do the child killing:
			char cmd[STR_SIZE];
			sprintf(cmd, "ps -o pid= --ppid %d", ForeignRelays[i].pid);			

			char buf[STR_SIZE];
			int child_pid;
			FILE * ptr;
			if((ptr = popen(cmd, "r")) != NULL)
			{
				fscanf(ptr, "%d", &child_pid);
				if(kill(child_pid, SIGTERM)!=0) ROS_WARN("Unable to kill foreign_relay child to %s with pid %d", ip, child_pid);
				if(kill(ForeignRelays[i].pid, SIGTERM)!=0) ROS_WARN("Unable to kill foreign_relay to %s with pid %d", ip, ForeignRelays[i].pid);
			}
			else
			{
				ROS_WARN("Unable to kill foreign_relay child to %s with ppid %d", ip, ForeignRelays[i].pid);
			}
			pclose(ptr);

			ROS_INFO("Closed foreign_relay to %s with pid %d", ip, ForeignRelays[i].pid);			

			ForeignRelays.erase(ForeignRelays.begin()+i);
		}
	}
	return;
}

void WiFiComm::neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg)
{
	unsigned int i, j, ind;
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
		// Trigger callback
		if(!found) (cbFunction)((char*)msg->neighbours[i].ip.c_str());
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

	// We store the msg so that we can access the list and size() without having to subscribe again
	neighboursList.self_ip = msg->self_ip;
	neighboursList.neighbours.clear();
	for(i=0 ; i<msg->neighbours.size() ; i++) neighboursList.neighbours.push_back(msg->neighbours[i]);
}

// EOF

