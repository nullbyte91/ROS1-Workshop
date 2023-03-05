#!/usr/bin/env python

import rospy
from custom_msg.msg import my_robot_msgs

def callback(data):
    rospy.loginfo("Received pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    
def subscriber():
    # Initialize the node
    rospy.init_node('pose_subscriber', anonymous=True)
    
    # Create a subscriber object with topic name, message type and callback function
    rospy.Subscriber('pose_topic', my_robot_msgs, callback)
    
    # Spin until the node is shut down
    rospy.spin()
        
if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
