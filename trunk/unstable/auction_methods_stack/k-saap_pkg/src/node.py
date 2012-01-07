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
import roslib; roslib.load_manifest('k-saap_pkg')

# import client library
import rospy

# import messages
import auction_msgs.msg

# import services
import auction_srvs.srv

# import services functions
import auctioneer
import buyer_k_saap

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
role_assigned = False



############################################################################
## Auction Config Service Callback
############################################################################
def handle_auction_server_callback(auction_req):

    global role_assigned

    # avoid node to take another role
    if not role_assigned:
        role_assigned = True
        if auction_req.role == 'be_auctioneer':
            return auctioneer.handle_auction_server_callback(auction_req)
        elif auction_req.role == 'be_buyer':
            return buyer_k_saap.handle_auction_server_callback(auction_req)
        else:
            return {'response_info':'invalid role requested'}
    else:
        return {'response_info':'node already have a role'}
## End handle_auction_config_server_callback 
      

                

#####################################################################################
## Main function
#####################################################################################
if __name__ == "__main__":
        
    # initialize node (we will have several nodes, anonymous=True)
    rospy.init_node('node', anonymous=True)
    
    # put service server into action
    service_path = rospy.get_name()+"/auction_server"

    auctioneer_response = rospy.Service(service_path,
                                     auction_srvs.srv.AuctionService,
                                     handle_auction_server_callback)

    # ok, ready to participate
    #rospy.loginfo(rospy.get_name()+" is ready to participate in the auction.")
    
    # Prevent node from exit until shutdown
    rospy.spin()

## End main
