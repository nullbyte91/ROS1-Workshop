#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received message: %s", data.data)
    
def subscriber():
    # Initialize the node
    rospy.init_node('string_subscriber', anonymous=True)
    
    # Create a subscriber object with topic name, message 
    # type and callback function
    rospy.Subscriber('string_topic', String, callback)
    
    # Spin until the node is shut down
    rospy.spin()
        
if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
