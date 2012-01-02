#! /usr/bin/env python

# The buyer role is played by the actors that can act on a particular overlapping area.

# Publish: Bid topic.
# Subscribe: Auction topic.
#            tf topic. (to do later)

# configuring PYTHONPATH (By default, this will add the src and lib directory for each of your dependencies to your PYTHONPATH)
import roslib; roslib.load_manifest('sap_pkg')

# import client library
import rospy

# import message types
import auction_msgs.msg

# import other libraries
import random


# defining callback for the auction announcement
# function: output information to rospy.loginfo
def auction_announcement_callback(data, bid_publisher):
    
    # output loginfo to rosout
    rospy.loginfo(rospy.get_name()+" %s started", data.command)

    if data.command == 'join_auction':
        # the auctioneer is accepting bids
        print "Auctioneer is accepting bids/n"

        # creates a bid (offer) to the item to be publish for negotiation
        bid_item = auction_msgs.msg.Bid()
        
        # fill the properties for the bid
        bid_item.header.frame_id = '0' # to be rechecked
        bid_item.header.stamp = rospy.Time.now()
        bid_item.buyer_id = rospy.get_name()
        bid_item.cost_distance = random.randint(0,100) # to be given by the cost to go to position xpto (for now is random)
        
        rospy.loginfo(bid_item)
        bid_publisher.publish(bid_item)
                
    else:
        pass



# defining the publication of an auction
# function: publish in Auction topic with command join_auction, and
#           the properties of the auction
def auction_announcement_listener():

    # declares that the node is publishing to the bids topic using the 
    # message type auction_msgs/Bid
    buyer_pub = rospy.Publisher('bids', auction_msgs.msg.Bid)

    # declares that the node is subscribing to the auction topic using the
    # message type auction_msgs/Auction
    rospy.Subscriber('auction', auction_msgs.msg.Auction, auction_announcement_callback, buyer_pub)

    rospy.sleep(0.5)
 

#
# define main part of the node
#
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the Buyer can
        # publish and subscribe over ROS.
        # There can be more than one buyer for the same auction
        rospy.init_node('buyer', anonymous=True)
        
        while not rospy.is_shutdown():
            # Publish the bid (if an auction is opened)
            auction_announcement_listener()
            
            # Blocks until ROS node is shutdown
            rospy.spin()            
            
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
