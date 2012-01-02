#! /usr/bin/env python

# The auctioneer role is played by the actor in charge of conducing the auction
# on a particular overlapping area.
# It is selected for each overlapping area by the collector/seller responsible
# for that area.

# Publish: Auction topic.
# Subscribe: Bid topic.

# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('sap_pkg')

# import client library
import rospy

# import message types
import auction_msgs.msg

# "global" variables (to be referred as global under def fun(something))
winner_id = 'none'
winner_cost = 0

# defining callback for the bids of an auction
# function: output information to rospy.loginfo
def auction_bids_callback(data):
    # define global variables
    global winner_id
    global winner_cost
    
    # output loginfo to rosout
    rospy.loginfo(rospy.get_name()+" %s offer %d", data.buyer_id, data.cost_distance)

    # evaluate bids (find the best)
    if winner_cost < data.cost_distance:
        winner_cost = data.cost_distance
        winner_id = data.buyer_id
    else:
        pass
    
    # verbose for auction status
    print "(winning at the moment) ", winner_id, "with offer ", winner_cost


# defining the publication of an auction
# function: publish in Auction topic with command join_auction, and
#           the properties of the auction
def auction_announcement():
    # reset variables (for new auction)
    global winner_id
    global winner_cost
    winner_id = "none"
    winner_cost = 0

    # declares that the node is publishing to the auction topic using the 
    # message type auction_msgs/Auction
    auctioneer_pub = rospy.Publisher("auction", auction_msgs.msg.Auction)

    # declares that the node is subscribing to the bids topic using the
    # message type auction_msgs/Bid
    bid = rospy.Subscriber("bids", auction_msgs.msg.Bid, auction_bids_callback)

    # wait a little bit here for Publisher and Subscribers to "connect"
    # to the topics
    rospy.sleep(0.5)

    # creates an auction item to be publish for negotiation
    auction_item = auction_msgs.msg.Auction()

    # fill the properties for the auction item
    auction_item.header.frame_id = "0"
    auction_item.header.stamp = rospy.Time.now()
    auction_item.command = "join_auction"
    auction_item.task_type_name = "go_to_location"
    auction_item.subject = "all"
    auction_item.metrics = "distance"
    auction_item.length = rospy.Duration(10) # in seconds

    # fill location where the event ocurred (to be automated)
    auction_item.task_location.x = 10.0
    auction_item.task_location.y = 10.0
    auction_item.task_location.z = 0.0 

    # publish auction on item
    rospy.loginfo(auction_item)
    auctioneer_pub.publish(auction_item)


    return auction_item
 

# defining the closing of an auction
# function: -subscribe Bid topic,
#           -evaluate for best offer, 
#           -publish in Auction topic with command close_auction, and
#            the properties of the auction
#           -notifies the winner
def auction_closing(auction_item):
    # close the Auction

    # declares that the node is publishing to the auction topic using the 
    # message type auction_msgs/Auction
    auctioneer_pub = rospy.Publisher("auction", auction_msgs.msg.Auction)    

    # change the command to close_auction
    auction_item.command = "close_auction"
    # publish close_auction command
    rospy.loginfo(auction_item)
    auctioneer_pub.publish(auction_item)

    # inform winner
    print "And the winner is ", winner_id," with offer ", winner_cost

   
#
# define main part of the node
#
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the Auctioneer can
        # publish and subscribe over ROS.
        # For now only one auctioneer is allowed
        rospy.init_node("auctioneer", anonymous=False)
        
#        while not rospy.is_shutdown():
        # Publish the auction (if an event is detected)
        auction_item_to_close = auction_announcement()
        
        # Time out (doesn't affect callback function)
        rospy.sleep(2.0)
        
        # Close the auction
        auction_closing(auction_item_to_close)
        
        # Blocks until ROS node is shutdown
#           rospy.spin()            
            
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
