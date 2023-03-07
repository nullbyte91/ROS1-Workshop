#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from robot_service.srv import DriveToTarget, DriveToTargetResponse

class Server:
    def __init__(self):
        _ = rospy.Service('/DriveToTarget', DriveToTarget, self.drive_bot)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.spin()
    def drive_bot(self, req):
        msg = Twist()
        msg.linear.x =  req.linear_x
        msg.angular.z =  req.angular_z
        self.pub.publish(msg)
        return DriveToTargetResponse(True)
        
def main():
    rospy.init_node('DriveToTarget_server')
    _ = Server()

if __name__ == "__main__":
    main()