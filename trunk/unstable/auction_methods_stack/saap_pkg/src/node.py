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

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
winner_id = 'none'
winner_cost = 0

role_assigned = False
node_role = 'none'



###################################################################################################
## Auction Client for Neighbour Nodes 
## (to be called in the node to pass data to its neighbours)
###################################################################################################
def neighbour_node_auction_client(neighbour_node, auction_req):

    # compose service name (to be changed)
    service_path = neighbour_node+'/auction_server'

    # wait for the service in the neighbour node to be available
    rospy.wait_for_service(service_path)

    try:
        # create the handle to the service client in the neighbour node
        neighbour_node_auction_server = rospy.ServiceProxy(service_path,
                                                           auction_srvs.srv.AuctionService)

        # call the service with the current auction information as input parameter
        neighbour_node_bid_response = neighbour_node_auction_server(auction_req)

        # log bid information from the neighbour node (debug)
        rospy.loginfo(neighbour_node_bid_response)

        # return the bid into the parent/calling node
        return neighbour_node_bid_response.bid_data
        
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s",e)
            
## End neighbour_node_auction_client





####################################################################################################
## Create list of neighbour nodes to relay the auction_req
## (must return a list)
####################################################################################################
def create_neighbour_nodes_list(auction_req):

    neighbour_nodes_string = rospy.get_param('~neighbour_nodes_list')
    neighbour_nodes_list = neighbour_nodes_string.split(',')

    ##debug##
    #print "1."
    #print neighbour_nodes_list
    #for node in neighbour_nodes_list:
    #    print node
    ##debug##

    #print neighbour_nodes_string
    #print auction_req.nodes_collected

    #nodes_collected_list = neighbour_nodes_list + auction_req.nodes_collected.split(',')
    #print "Collected nodes list:"
    #print nodes_collected_list

    # print "Intersection:"
    # print list(set(neighbour_nodes_list) & set(auction_req.nodes_collected.split(',')))
    # print "Union:"
    # print list(set(neighbour_nodes_list) | set(auction_req.nodes_collected.split(',')))
    # print "Difference"
    # print list(set(neighbour_nodes_list) - set(auction_req.nodes_collected.split(',')))

    nodes_collected_list = list(set(neighbour_nodes_list) - set(auction_req.nodes_collected.split(',')))
    

    # remove '' strings
    while '' in nodes_collected_list:
        nodes_collected_list.remove('')

    # remove duplicates
    nodes_collected_list = list(set(nodes_collected_list))
    
    # remove self-references
    while rospy.get_name() in nodes_collected_list:
        nodes_collected_list.remove(rospy.get_name())

    # remove references to the sender node
    while auction_req.sending_node in nodes_collected_list:
        nodes_collected_list.remove(auction_req.sending_node)
        

    if nodes_collected_list:

        # convert list to string splited by ','
        nodes_collected_string = ','.join(nodes_collected_list)

        ##debug##
        #print "\nNodes Collected:"+nodes_collected_string+"\n"
        ##debug##

        neighbour_nodes_list = nodes_collected_string.split(',')

    else:
        neighbour_nodes_list = []
        pass

    return neighbour_nodes_list
    #return nodes_collected_list

## End create_neighbour_nodes_list


def auctioneer():
    pass

def buyer():
    pass


####################################################################################################
## Auction Service (Server Callback)
####################################################################################################
def handle_auction_server_callback(auction_req):

    # define global variables
    global winner_id
    global winner_cost    
    global role_assigned
    global node_role

    # rospy log the requested role
    rospy.loginfo("the node: "+rospy.get_name()+" was asked to: %s",auction_req.role)

    # check for auction_req.auction_data.command (if close_auction > clear role assignment)
    if auction_req.auction_data.command == 'close_auction':
        role_assigned = False
        node_role = 'none'

        # Calculates its own bid offer for the item in auction_req
        return {'response_info': 'invalid_bid'}

    elif auction_req.auction_data.command == 'join_auction':

        # if the node has already a role does nothing
        if role_assigned:
            rospy.loginfo("I'm node: "+rospy.get_name()+" and i'm already a(n) %s",node_role)

            return {'response_info': 'invalid_bid'}
            
        # if the node doesn't have a role assigned to it
        else:          
        
            #################
            # if auctioneer #
            #################
            if auction_req.role == "be_auctioneer":       
                rospy.loginfo("-->I'm "+rospy.get_name()+" and i'm an auctioneer!<--")

                # change role_assigned flag to avoid re-assignment in the same auction
                # causing infinite loop
                role_assigned = True
                node_role = 'Auctioneer'+rospy.get_name()
                
                # Obtain nodes list to relay information
                neighbour_nodes_relay_list = create_neighbour_nodes_list(auction_req)
                
                # Change sending_node in auction_req to be sent to neighbour nodes
                auction_req.sending_node = rospy.get_name()

                # Change nodes_collected in auction_req to be sent to neighbour nodes
                auction_req.nodes_collected = rospy.get_param('~neighbour_nodes_list')
            
                # Change the role in auction_req to be sent to neighbour nodes
                auction_req.role = "be_buyer"
            
                # Call the Auction Service from each neighbour node
                for node in neighbour_nodes_relay_list:

                    # obtain response from neighbour buyer node (in k=1), relaying auction_req
                    bid_response = neighbour_node_auction_client(node, auction_req)
                
                    # Evaluate bids (the auctioneer will return the winning bid), Min(cost_distance)
                    if winner_id == 'none':
                        winner_cost = bid_response.cost_distance
                        winner_id = bid_response.buyer_id
                
                    elif winner_cost >= bid_response.cost_distance:
                        winner_cost = bid_response.cost_distance
                        winner_id = bid_response.buyer_id
                
                    # log info for momentary winner
                    rospy.loginfo("(winning at the moment) %s with offer %d",winner_id, winner_cost)

                # verbose for auction status (received all the bids)
                rospy.loginfo("winner was: %s with offer %d",winner_id, winner_cost)


                # (close auction and inform winner) 
                # (client to neighbour nodes with close_auction req)
                # (in close_auction req the nodes reset their roles!!)

                # Change the command in auction_req to be sent to neighbour nodes
                auction_req.auction_data.command = "close_auction"

                # Call the Auction Service Reset from each neighbour node
                for node in neighbour_nodes_relay_list:
                
                    # obtain response from neighbour buyer node (in k=1), relaying auction_req
                    reset_response = neighbour_node_auction_client(node, auction_req)

                # Ok, now we can reset our role
                role_assigned = False
                node_role = 'none'

                #return{'',''}
            
            ############        
            # if buyer #
            ############       
            elif auction_req.role =="be_buyer":
                rospy.loginfo("-->I'm "+rospy.get_name()+" and i'm a buyer!<--")

                # change role_assigned flag to avoid re-assignment in the same auction,
                # causing infinite loop
                role_assigned = True
                node_role = 'Buyer'+rospy.get_name()

                # Create a bid messsage to put an offer for the item in auction_req
                bid_response = auction_msgs.msg.Bid()
                # Fill up the fields in the bid offer
                bid_response.header.frame_id = 'base_link' # to be rechecked
                bid_response.header.stamp = rospy.Time.now()
                bid_response.buyer_id = rospy.get_name()          
            
                if auction_req.auction_data.metrics == "distance":
                    # to be given by the cost to go to position of the ocurring event
                    # the cost for the metrics==distance is calculated using the euclidean
                    # distance between the parameter position of the node and the task_position
                    # given in the auction_req
                    node_position = eval(rospy.get_param('~position'))
                    x = int(node_position[0])-auction_req.auction_data.task_location.x
                    y = int(node_position[1])-auction_req.auction_data.task_location.y
                    z = int(node_position[2])-auction_req.auction_data.task_location.z
                    bid_response.cost_distance = int(math.sqrt(x*x+y*y+z*z))
                else:
                    rospy.loginfo("Metrics unkown")
                    bid_response.const_distance = 1000;
            
            
                # Evaluate the type of auction and proceed according to it!
                    
                # if auction_type is 'sap' (Simple Auction Protocol)
                if auction_req.auction_type == 'sap':
                
                    # Obtain nodes list to relay information auction_req
                    neighbour_nodes_relay_list = create_neighbour_nodes_list(auction_req)

                    # If the list return not empty
                    if neighbour_nodes_relay_list:

                        # Change sending_node in auction_req to be sent to neighbour nodes
                        auction_req.sending_node = rospy.get_name()
                    
                        # Change nodes_collected (update) in auction_req to be sent to neighbours
                        auction_req.nodes_collected = rospy.get_param('~neighbour_nodes_list')

                        # call the Auction Service from each neighbour of the node
                        for node in neighbour_nodes_relay_list:

                            # obtain response from neighbour buyer node (in k=1)                 
                            bid_response_neighbour_node = neighbour_node_auction_client(node, 
                                                                                        auction_req)
                    else :
                        return {'response_info': 'valid', 'bid_data':bid_response}
                
                    # Publish Bid (topic?? - probably not)  

                    # Compare own bid with neighbour bid (later)
            
                # if auction_type is 'k-sap' (k-Simple Auction Protocol)
                elif auction_req.auction_type == 'k-sap':
                    pass
                # if auction_type is 'saap' (Simple Aggregation Auction Protocol)
                elif auction_req.auction_type == 'sap':
                    pass
                # if auction_type is 'k-saap' (k-Simple Aggregation Auction Protocol)
                elif auction_req.auction_type == 'sap':
                    pass

    # return response
    # return auction_srvs.srv.AuctionServiceResponse(bid_response)
    return {'response_info': 'valid', 'bid_data':bid_response}
## End Auction Server (Server Callback)



          
#####################################################################################################
## Auction Service (Server)
#####################################################################################################
def auction_server():

    service_path = rospy.get_name()+"/auction_server"
    
    auction_response = rospy.Service(service_path, 
                                     auction_srvs.srv.AuctionService, 
                                     handle_auction_server_callback)
    
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
