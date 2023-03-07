#!/usr/bin/env python

import rospy
from enum import Enum
from sensor_msgs.msg import Image
from robot_service.srv import DriveToTarget, DriveToTargetRequest

class Side(Enum):
    LEFT = 0
    FORWARD = 1
    RIGHT = 2
    NO_BALL = 3

class ProcessImage:
    def __init__(self):
        rospy.wait_for_service('/DriveToTarget')
        self.side = Side.NO_BALL

        # Initialize ROS node
        rospy.init_node('process_image')

        # Define a client service capable of requesting services from command_robot
        self.client = rospy.ServiceProxy('/DriveToTarget', DriveToTarget)

        # Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image_callback)

        # Handle ROS communication events
        rospy.spin()

    def drive_robot(self, lin_x, ang_z):
        # Call the command_robot service and pass the requested joint angles
        try:
            self.client(lin_x, ang_z)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service command_robot: %s" % e)

    def process_image_callback(self, img):
        # Determine if there is a white ball and if yes, on which side
        for i in range(0, img.height * img.step, 3):

            red_channel = img.data[i]
            green_channel = img.data[i+1]
            blue_channel = img.data[i+2]

            if red_channel == 255 and green_channel == 255 and blue_channel == 255:
                print(red_channel)
                print(green_channel)
                print(blue_channel)
                pixel_val = i % img.step
                if pixel_val < img.step * 0.4:
                    rospy.loginfo("Left value: %s" % pixel_val)
                    self.side = Side.LEFT
                elif pixel_val > img.step * 0.6:
                    rospy.loginfo("Right value: %s" % pixel_val)
                    self.side = Side.RIGHT
                else:
                    rospy.loginfo("Forward value: %s" % pixel_val)
                    self.side = Side.FORWARD
                break
            else:
                self.side = Side.NO_BALL

        # Drive robot towards the ball
        if self.side == Side.LEFT:
            self.drive_robot(0.5, 1.0)
        elif self.side == Side.RIGHT:
            self.drive_robot(0.5, -1.0)
        elif self.side == Side.FORWARD:
            self.drive_robot(0.5, 0.0)
        elif self.side == Side.NO_BALL:
            self.drive_robot(0.0, 0.0)

if __name__ == '__main__':
    process_image = ProcessImage()
