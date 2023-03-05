#!/usr/bin/env python
import rospy # Python client lib for ROS

rospy.init_node("hello_world")     # Initiate a node called hello_world
rate = rospy.Rate(1)               # We create a Rate object of 1Hz
while not rospy.is_shutdown():     # Continous loop
   print("My first ROS package Hello world ")
   rate.sleep()                    # We sleep the needed time to maintain the above Rate