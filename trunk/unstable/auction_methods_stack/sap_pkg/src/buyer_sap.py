# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('sap_pkg')

# import client library
import rospy

# import messages
import auction_msgs.msg

# import services
import auction_srvs.srv

import auction_common

# import auxiliar libraries
import random
import math

# "global" variables (to be referred as global under def fun(something))
winner_id = 'none'
winner_cost = 0

role_assigned = False
node_role = 'none'




def handle_auction_server_callback(auction_req):
    
    ############        
    # if buyer #
    ############       
    if auction_req.role =="be_buyer":
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
            x = float(node_position[0])-auction_req.auction_data.task_location.x
            y = float(node_position[1])-auction_req.auction_data.task_location.y
            z = float(node_position[2])-auction_req.auction_data.task_location.z
            bid_response.cost_distance = float(math.sqrt(x*x+y*y+z*z))
        else:
            rospy.loginfo("Metrics unkown")
            bid_response.const_distance = 1000;
            
            
            # Obtain nodes list to relay information auction_req
            neighbour_nodes_relay_list = auction_common.create_neighbour_nodes_list(auction_req)
            
            # If the list return not empty
            if neighbour_nodes_relay_list:
                
                # Change sending_node in auction_req to be sent to neighbour nodes
                auction_req.sending_node = rospy.get_name()
                
                # Change nodes_collected (update) in auction_req to be sent to neighbours
                auction_req.nodes_collected = rospy.get_param('~neighbour_nodes_list')
                
                # call the Auction Service from each neighbour of the node
                for node in neighbour_nodes_relay_list:
                    
                    # obtain response from neighbour buyer node (in k=1)                 
                    bid_response_neighbour_node = auction_common.neighbour_node_auction_client(node,auction_req)
                else :
                    return {'response_info': 'valid', 'bid_data':bid_response}
                
                # Publish Bid (topic?? - probably not)  
                
                # Compare own bid with neighbour bid (later)
                
    return {'response_info': 'valid', 'bid_data':bid_response}
