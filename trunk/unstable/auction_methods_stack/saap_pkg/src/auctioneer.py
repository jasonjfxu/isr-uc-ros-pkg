# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('saap_pkg')

# import client library
import rospy

# import messages
import auction_msgs.msg

# import services
import auction_srvs.srv

# import services functions
import auction_common

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
winner_id = ''
winner_cost = 999999


#####################################################################################
## Auction Service (Server Callback)
#####################################################################################
def handle_auction_server_callback(auction_req):

    # define global variables
    global winner_id
    global winner_cost    

    # check for auction_req.auction_data.command (if close_auction -> clear role assignment
    if auction_req.auction_data.command == 'close_auction':
        role_assigned = False

        # Calculates its own bid offer for the item in auction_req
        return {'response_info': 'invalid_bid'}

    elif auction_req.auction_data.command == 'join_auction':         
        
        # Obtain nodes list to relay information with k=1
        neighbour_nodes_relay_list = auction_common.create_neighbour_nodes_list(auction_req)        
        
        # Change sending_node in auction_req to be sent to neighbour nodes
        auction_req.sending_node = rospy.get_name()
        # Change nodes_collected in auction_req to be sent to neighbour nodes
        auction_req.nodes_collected = rospy.get_param('~neighbour_nodes_list')          
        # Change the role in auction_req to be sent to neighbour nodes
        auction_req.role = "be_buyer"
            
        # Call the Auction Service from each neighbour node
        for node in neighbour_nodes_relay_list:
            # obtain response from neighbour buyer node (in k=1), relaying auction_req
            bid_response = auction_common.neighbour_node_auction_client(node, auction_req)

            # Evaluate bids, Min(cost_distance)    
            if winner_cost >= bid_response.cost_distance:
                if bid_response.buyer_id != '':
                    winner_cost = bid_response.cost_distance
                    winner_id = bid_response.buyer_id
                
                # log info for momentary winner
                # rospy.loginfo("(winning at the moment) %s with offer %d",winner_id, winner_cost)
                
        # verbose for auction status (received all the bids)
        rospy.loginfo("winner was: %s with offer %d",winner_id, winner_cost)

        """
        # (close auction and inform winner) 
        # (client to neighbour nodes with close_auction req)
        # (in close_auction req the nodes reset their roles!!)
        
        # Change the command in auction_req to be sent to neighbour nodes
        auction_req.auction_data.command = "close_auction"
                
        # Call the Auction Service Reset from each neighbour node
        for node in neighbour_nodes_relay_list:
                
            # obtain response from neighbour buyer node (in k=1), relaying auction_req
            reset_response = auction_common.neighbour_node_auction_client(node, auction_req)

            # Ok, now we can reset our role
            role_assigned = False
            node_role = 'none'
            
            
        return {'response_info': 'valid', 'bid_data':bid_response}
        """           
    # return response
    # return auction_srvs.srv.AuctionServiceResponse(bid_response)
    return {'response_info': 'valid', 'bid_data':bid_response}
## End Auction Server (Server Callback)
