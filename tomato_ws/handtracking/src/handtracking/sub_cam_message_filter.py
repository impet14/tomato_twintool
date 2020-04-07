#!/usr/bin/env python
import threading
import sys
import message_filters ### ADD THIS
import rospy
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
# import pyrealsense2 as rs
# from utils import detector_utils as detector_utils

cv_image_rgb = np.zeros((360,640,3), np.uint8)

#------Start of Class
class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        image_align_hand_color = "/camera/color/image_rect_color"
        image_align_hand_depth = "/camera/aligned_depth_to_color/image_raw"
        self.image_rgb_sub = message_filters.Subscriber(image_align_hand_color,Image)    #Case 1
        self.image_depth_sub = message_filters.Subscriber(image_align_hand_depth,Image)  #Case 2       
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_rgb_sub, self.image_depth_sub],1,2, allow_headerless=True)
        self.ts.registerCallback(self.callback)

    def callback(self, rgb,depth):
        if not rgb or not depth:
            print ("image not completed")
            return

        cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb, "passthrough")
        cv_image_depth = self.bridge.imgmsg_to_cv2(depth,"16UC1")
        cv_image_depth = np.array(cv_image_depth, dtype=np.uint8)

        cv2.imshow("RGB_Sub", cv_image_rgb)
        # print rgb.encoding

        
        # print depth.encoding
        cv2.imshow("Depth_Sub", cv_image_depth)

        cv2.waitKey(10)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     rospy.signal_shutdown('Quit')
        #     cv2.destroyAllWindows()


def main():
    # global proc_id
    # proc_id = sys.argv[1]
    # rospy.init_node(proc_id)
    img_cvt = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Hello ROS!")
    try:        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

# Reference
# http://arktechai.blogspot.com/2017/03/using-opencv-to-display-intel-realsense.html
# https://answers.ros.org/question/312166/how-to-subcribe-2-image-topics-with-1-node-in-same-time/
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/