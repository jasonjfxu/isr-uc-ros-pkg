#! /usr/bin/env python

# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('sap_pkg')

# import client library
import rospy

# import messages
import auction_msgs.msg

# import services
import auction_srvs.srv

# import auxiliar libraries
import random
import math
import sys


###############################################################################
## Main function
###############################################################################
if __name__ == "__main__":
        
    # initialize node (we will have several nodes, anonymous=True)
    rospy.init_node('seller', anonymous=False)

    # create auction information
    role = 'be_auctioneer'
    auction_type = 'sap'
    sending_node = rospy.get_name()
    nodes_collected = ''
  
    auction_data = auction_msgs.msg.Auction()
    auction_data.header.stamp = rospy.Time.now()
    auction_data.header.frame_id = 'base_link'
    auction_data.command = 'join_auction'
    auction_data.task_type_name = 'goto'
    auction_data.subject = 'all'
    auction_data.metrics = 'distance'
    auction_data.length = rospy.Duration(10)
    auction_data.task_location.x = random.random()*100 # l = 100
    auction_data.task_location.y = random.random()*100 # l = 100
    auction_data.task_location.z = 0

    print auction_data.task_location


    # evaluate nearest node to event position
    event_location = auction_data.task_location
    stop_search = False
    i = 1
    nearest_node = []
    nearest_node_distance = 99999999
    while not stop_search:
        try:
            node = '/node_'+str(i)
            node_param = node+'/position'
            node_position = eval(rospy.get_param(node_param))

            # calculate distance to event
            x = float(node_position[0])-event_location.x
            y = float(node_position[1])-event_location.y
            z = float(node_position[2])-event_location.z
            distance = float(math.sqrt(x*x+y*y+z*z))

            if distance < nearest_node_distance:
                nearest_node = node
                nearest_node_position = node_position
                nearest_node_distance = distance

            i+=1
        except:
            rospy.loginfo("Node evaluation complete. Node %s will be auctioneer",nearest_node)
            stop_search = True


    # prepare to call AuctionConfigService in the node that will be the auctioneer
    service_path = nearest_node+'/auction_config_server'
    
    rospy.wait_for_service(service_path)
    auction_config_service = rospy.ServiceProxy(service_path, auction_srvs.srv.AuctionConfigService)

    try:
        auction_config_server_resp = auction_config_service(role,auction_type,sending_node)
        rospy.loginfo(auction_config_server_resp)
    except rospy.ServiceException, e:
        rospy.logwarn("Service did not process request: %s",e)


    # send the auction information to the auctioneer node using the AuctioneerService
    service_path = nearest_node+'/auctioneer_server'   
    
    rospy.wait_for_service(service_path)
    auctioneer_service = rospy.ServiceProxy(service_path, auction_srvs.srv.AuctioneerService)
    rospy.loginfo("[message] Seller request")

    try:
        auctioneer_server_resp = auctioneer_service(sending_node,nodes_collected,auction_data)
        rospy.loginfo("[message] Auctioneer response")
        rospy.loginfo(auctioneer_server_resp)
    except rospy.ServiceException, e:
        rospy.logwarn("Service did not process request: %s",e)


    rospy.set_param('/auction_closed', True)
                                                     
## End main
