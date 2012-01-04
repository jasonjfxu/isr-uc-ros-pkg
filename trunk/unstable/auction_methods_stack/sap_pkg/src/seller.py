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
    auction_data.task_location.x = 0 #random.random()*100 # l = 100
    auction_data.task_location.y = 0 #random.random()*100 # l = 100
    auction_data.task_location.z = 0

    print auction_data.task_location


    # prepare to call AuctionConfigService in the node that will be the auctioneer
    service_path = sys.argv[1]+'/auction_config_server'   
    
    rospy.wait_for_service(service_path)
    auctioneer_service = rospy.ServiceProxy(service_path, auction_srvs.srv.AuctionConfigService)

    try:
        auction_config_server_resp = auctioneer_service(role,auction_type,sending_node)

    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)


    # send the auction information to the auctioneer node using the AuctioneerService
    service_path = sys.argv[1]+'/auctioneer_server'   
    
    rospy.wait_for_service(service_path)
    auctioneer_service = rospy.ServiceProxy(service_path, auction_srvs.srv.AuctioneerService)

    try:
        auctioneer_server_resp = auctioneer_service(sending_node,nodes_collected,auction_data)

    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)

                                                     
## End main
