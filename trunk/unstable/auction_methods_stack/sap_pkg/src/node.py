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
import auctioneer
import buyer_sap

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
role_assigned = False


############################################################################
## Auctioneer Services
############################################################################
def auctioneer_services():

    if rospy.has_param('/auction_close'):
        if rospy.get_param('/auction_close') == True:
            auctioneer_server.shutdown('auction_closing')
            auctioneer_bid_reception_server.shutdown('auction_closing')

    # create Auctioneer Service
    service_path = rospy.get_name()+"/auctioneer_server"

    auctioneer_server = rospy.Service(service_path, auction_srvs.srv.AuctioneerService, auctioneer.handle_auctioneer_server_callback)
    

    # create Auctioneer Bid Reception Service
    service_path = rospy.get_name()+"/auctioneer_bid_reception_server"

    auctioneer_bid_reception_server = rospy.Service(service_path, auction_srvs.srv.AuctioneerBidReceptionService, auctioneer.handle_auctioneer_bid_reception_server_callback)


    #if rospy.has_param('/auction_close'):
    #    if rospy.get_param('/auction_close') == True:
    #        auctioneer_server.shutdown('auction_closing')
    #        auctioneer_bid_reception_server.shutdown('auction_closing')
    
## End create_auctioneer_services



############################################################################
## Buyer Services
############################################################################
def buyer_services():

    if rospy.has_param('/auction_close'):
        if rospy.get_param('/auction_closed') == True:
            buyer_server.shutdown('auction_closing')

    # create Buyer Service
    service_path = rospy.get_name()+"/buyer_server"
    
    buyer_server = rospy.Service(service_path, auction_srvs.srv.BuyerService, buyer_sap.handle_buyer_server_callback)

    #if rospy.has_param('/auction_close'):
    #    if rospy.get_param('/auction_closed') == True:
    #        buyer_server.shutdown('auction_closing')
    
## End create_buyer_services



############################################################################
## Auction Config Service Callback
############################################################################
def handle_auction_config_server_callback(auction_req):

    global role_assigned

    if rospy.has_param('/auction_closed'):
        if rospy.get_param('/auction_closed') == True:
            role_assigned = False

    rospy.loginfo(rospy.get_name()+' '+str(role_assigned))

    # avoid node to take another role
    if not role_assigned:
        role_assigned = True

        if auction_req.role == 'be_auctioneer':
            auctioneer_services()
            return {'response_info':'Auctioneer: '+rospy.get_name()+' ready for auction'}
        
        elif auction_req.role == 'be_buyer':
            buyer_services()
            return {'response_info':'Buyer: '+rospy.get_name()+' ready for auction'}
        
        else:
            return {'response_info':'invalid role requested'}


        if rospy.has_param('/auction_closed'):
            rospy.set_param('/auction_closed', False)

    else:
        return {'response_info':'node already have a role'}

## End handle_auction_config_server_callback            



############################################################################
## Main function
############################################################################
if __name__ == "__main__":
        
    # initialize node (we will have several nodes, anonymous=True)
    rospy.init_node('node', anonymous=True)
    

    # create the auction_config_server
    service_path = rospy.get_name()+"/auction_config_server"


    auction_config_server = rospy.Service(service_path,
                                          auction_srvs.srv.AuctionConfigService,
                                          handle_auction_config_server_callback)
    
    
    # ok, ready to participate
    # rospy.loginfo(rospy.get_name()+" is ready to participate in the auction.")

    # Prevent node from exit until node shutdown
    rospy.spin()

## End main
