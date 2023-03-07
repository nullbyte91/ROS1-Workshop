#!/usr/bin/env python3
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ros_service.srv import localizeImg, localizeImgResponse
import rospy
import time
import random as rand
class Client:
    def __init__(self):
        rospy.wait_for_service('localize')
        self.bridge = CvBridge()
        self.plan_and_getImage()

    def plan_and_getImage(self):
        frequency = rospy.get_param("/image_acquisition/frequency")
        rate = rospy.Rate(frequency)
        while True:
            localizeImage = rospy.ServiceProxy('localize', localizeImg)
            x = int(rand.random())
            y = int(rand.random())
            image_msg = localizeImage(x, y)
            cv_image = self.bridge.imgmsg_to_cv2(image_msg.image, "bgr8")
            cv2.imshow("Display", cv_image)
            cv2.waitKey(1)
            rate.sleep()

def main():
    rospy.init_node('client')
    _ = Client()

if __name__ == "__main__":
    main()