import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
from ros_service.srv import localizeImg, localizeImgResponse

class Server:
    def __init__(self):
        self.bridge = CvBridge()
        self.video_capture = None
        self.cam_init()
        _ = rospy.Service('localize', localizeImg, self.pathPlanning)
        rospy.spin()

    def cam_init(self):
        input_type = rospy.get_param("/image_acquisition/input_type")
        if input_type == "camera_usb":
            self.video_capture  = cv2.VideoCapture(0)
        else:
            video_path = rospy.get_param("/image_acquisition/video/video_path_0")
            self.video_capture  = cv2.VideoCapture(video_path)
    
    def pathPlanning(self, req):
        print("Move robot to x:{} y:{} position".format(req.x, req.y))
        time.sleep(2)
        _, frame = self.video_capture.read()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            return msg

def main():
    rospy.init_node('localize_server')
    _ = Server()

if __name__ == "__main__":
    main()