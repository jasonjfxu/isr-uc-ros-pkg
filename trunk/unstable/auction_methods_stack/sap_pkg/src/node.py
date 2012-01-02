#! /usr/bin/env python

# Each node in the network can:
#   - be called to serve as actioneer or buyer;
#   - actioneer service is called by a seller/collector node with 
#     request start_auction;
#   - buyer service is called by the a auctioneer node with request
#     join_auction / close_auction

# The node will need to run a general server service, that will redirect to the
# correct service callback action according to the received request 
# (e.g. seller node request the auctioneer service sending start_auction 
# message and the node will provide the auctioneer service; 
# auctioneer node request the buyer service sending join_auction message 
# and the node will provide the buyer service 


# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('sap_pkg')

# import client library
import rospy

# import messages
import auction_msgs.msg

# import services
import auction_srvs.srv

# import services functions
import auctioneer_sap
import buyer_sap

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
winner_id = 'none'
winner_cost = 0

role_assigned = False
node_role = 'none'

         
#####################################################################################################
## Auction Service (Server)
#####################################################################################################
def auction_server():

    # Auctioneer Service
    service_path = rospy.get_name()+"/auctioneer_server"

    auctioneer_response = rospy.Service(service_path,
                                     auction_srvs.srv.AuctionService, #attention here (type of service)
                                     auctioneer_sap.handle_auction_server_callback)

    # Buyer Service
    service_path = rospy.get_name()+"/buyer_server"
    
    buyer_response = rospy.Service(service_path, 
                                   auction_srvs.srv.AuctionService, #attention here (type of service)
                                   buyer_sap.handle_auction_server_callback)    

    # ok, ready to participate
    rospy.loginfo(rospy.get_name()+" is ready to participate in the auction.")
    
    # Prevent node from exit until shutdown
    rospy.spin()
## End Auction Service (Server)
                

#####################################################################################################
## Main function
#####################################################################################################
if __name__ == "__main__":
        
    # initialize node (we will have several nodes, anonymous=True)
    rospy.init_node('node', anonymous=True)
    
    # put service server into action
    auction_server()
## End main
