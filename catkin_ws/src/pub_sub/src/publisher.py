#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    # Initialize the node
    rospy.init_node('string_publisher', anonymous=True)
    
    # Create a publisher object with topic name, message type and queue size
    pub = rospy.Publisher('string_topic', String, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(10)
    
    # Publish messages until the node is shut down
    while not rospy.is_shutdown():
        # Create a message
        message = String()
        message.data = "Hello, World!"
        
        # Publish the message
        pub.publish(message)
        
        # Sleep to maintain the loop rate
        rate.sleep()
        
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
