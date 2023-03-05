#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Subscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber("publisher", Image, self.callback)
        self.bridge = CvBridge()
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Display", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('subscriber')
    Subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)