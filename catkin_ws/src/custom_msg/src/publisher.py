#!/usr/bin/env python

import rospy
from custom_msg.msg import my_robot_msgs

def publisher():
    # Initialize the nodes
    rospy.init_node('pose_publisher', anonymous=True)
    
    # Create a publisher object with topic name, message type and queue size
    pub = rospy.Publisher('pose_topic', my_robot_msgs, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(10)
    
    # Publish messages until the node is shut down
    while not rospy.is_shutdown():
        # Create a message
        message = my_robot_msgs()
        message.position.x = rospy.get_param("/Pose/position/x")
        message.position.y = rospy.get_param("/Pose/position/y")
        message.position.z = rospy.get_param("/Pose/position/z")
        message.orientation.x = rospy.get_param("/Pose/orientation/x")
        message.orientation.y = rospy.get_param("/Pose/orientation/y")
        message.orientation.z = rospy.get_param("/Pose/orientation/z")
        message.orientation.w = rospy.get_param("/Pose/orientation/w")
        
        # Publish the message
        pub.publish(message)
        
        # Sleep to maintain the loop rate
        rate.sleep()
        
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
