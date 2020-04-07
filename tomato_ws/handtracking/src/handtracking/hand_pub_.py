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
from utils import detector_utils as detector_utils
import cv2
import tensorflow as tf
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
import pyrealsense2 as rs
import argparse
import datetime

#get intricsic of cam manually
rs_image_rgb = np.zeros((360,640,3), np.uint8)
rs_image_depth = np.zeros(rs_image_rgb.shape, np.uint16)
new_image = False

#get them from realsense api 
depth_scale = 0.0010000000474974513
depth_intrin = rs.intrinsics()
depth_intrin.width = rs_image_rgb.shape[1]
depth_intrin.height = rs_image_rgb.shape[0]
depth_intrin.coeffs = [0, 0, 0, 0, 0]
depth_intrin.fx = 461.9239807128906
depth_intrin.fy = 462.3507080078125
depth_intrin.ppx = 321.3486328125
depth_intrin.ppy = 179.70550537109375
depth_intrin.model = rs.distortion.brown_conrady
# print depth_intrin

#init detecting session
detection_graph, sess = detector_utils.load_inference_graph()
#------Start of realsense camera class
class rs_process:
    def __init__(self):
        self.bridge = CvBridge()
        image_align_hand_color = "/camera/color/image_rect_color"
        image_align_hand_depth = "/camera/aligned_depth_to_color/image_raw"
        self.image_sub = rospy.Subscriber(image_align_hand_color, Image, self.color_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(image_align_hand_depth, Image, self.depth_callback, queue_size=1)
    #   
    def color_callback(self, data):
        global rs_image_rgb
        global new_image
        new_image = True

        # print (data.encoding) #rgb8
        try:
            rs_image_rgb = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
    #
    def depth_callback(self, data):
        global rs_image_depth
        # print(data.encoding) #16uc1
        try:
            rs_image_depth = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
    #
# 
def main():

    global new_image

    rs_img = rs_process()
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("Hand Detection Start!")

    #Marker Publisher Initialize
    pub=rospy.Publisher('/hand_marker', Marker)
    hand_mark = MarkerGenerator()
    hand_mark.type = Marker.SPHERE_LIST
    hand_mark.scale = [.04]*3
    hand_mark.frame_id = '/camera_color_optical_frame'
    # hand_mark.frame_id = 'iiwa_link_0'

    #hand detect args
    parser = argparse.ArgumentParser()
    parser.add_argument('-sth', '--scorethreshold', dest='score_thresh', type=float,
                        default=0.2, help='Score threshold for displaying bounding boxes')
    parser.add_argument('-fps', '--fps', dest='fps', type=int,
                        default=1, help='Show FPS on detection/display visualization')
    parser.add_argument('-src', '--source', dest='video_source',
                        default=0, help='Device index of the camera.')
    parser.add_argument('-wd', '--width', dest='width', type=int,
                        default=640, help='Width of the frames in the video stream.')
    parser.add_argument('-ht', '--height', dest='height', type=int,
                        default=360, help='Height of the frames in the video stream.')
    parser.add_argument('-ds', '--display', dest='display', type=int,
                        default=0, help='Display the detected images using OpenCV. This reduces FPS')
    parser.add_argument('-num-w', '--num-workers', dest='num_workers', type=int,
                        default=4, help='Number of workers.')
    parser.add_argument('-q-size', '--queue-size', dest='queue_size', type=int, 
                        default=5, help='Size of the queue.')
    args = parser.parse_args()
    num_hands_detect = 2

    im_width, im_height =  (args.width, args.height)

    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0

    #skin filter color
    lower = np.array([0, 48, 80], dtype = "uint8")
    upper = np.array([20, 255, 255], dtype = "uint8")

    while not rospy.is_shutdown():
        #get rgb,depth frames for synchronized frames
        if not new_image:
            continue

        im_rgb = rs_image_rgb
        im_depth = rs_image_depth
        new_image = False
        #add check


        # depth_map = np.array(rs_image_depth, dtype=np.uint8)
        depth_map = cv2.applyColorMap(cv2.convertScaleAbs(rs_image_depth, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow("Depth Image", depth_map)
        cv2.imshow("Color Image", rs_image_rgb) 

        try:
            image_np = im_rgb #cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)
        except:
            print("Error converting to RGB")

        # cv2.imshow("source image np", image_np)

        # actual hand detection
        boxes, scores = detector_utils.detect_objects(
            image_np, detection_graph, sess)
        # draw bounding boxes
        detector_utils.draw_box_on_image(
            num_hands_detect, args.score_thresh, scores, boxes, im_width, im_height, image_np)

        if (scores[0] > args.score_thresh):
            (left, right, top, bottom) = (boxes[0][1] * im_width, boxes[0][3] * im_width,
                                          boxes[0][0] * im_height, boxes[0][2] * im_height)
            p1 = (int(left), int(top))
            p2 = (int(right), int(bottom))
            print p1,p2,int(left),int(top),int(right),int(bottom)
            image_hand = image_np[int(top):int(bottom), int(left):int(right)]
            cv2.namedWindow("hand", cv2.WINDOW_NORMAL)
            cv2.imshow('hand', cv2.cvtColor(image_hand, cv2.COLOR_RGB2BGR))

            align_hand = im_rgb[int(top):int(bottom), int(left):int(right)]
            align_depth = depth_map[int(top):int(bottom),int(left):int(right)]
            align_hand_detect = np.hstack((align_hand, align_depth))
            cv2.namedWindow('align hand', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('align hand', align_hand_detect)

            #skin filtering
            converted = cv2.cvtColor(align_hand, cv2.COLOR_BGR2HSV)
            skinMask = cv2.inRange(converted, lower, upper)
         
            # apply a series of erosions and dilations to the mask
            # using an elliptical kernel
            # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
            # skinMask = cv2.erode(skinMask, kernel, iterations = 2)
            # skinMask = cv2.dilate(skinMask, kernel, iterations = 2)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            skinMask = cv2.erode(skinMask, kernel, iterations = 1)
            skinMask = cv2.dilate(skinMask, kernel, iterations = 1)
         
            # blur the mask to help remove noise, then apply the
            # mask to the frame
            skinMask = cv2.GaussianBlur(skinMask, (3, 3), 0)
            skin = cv2.bitwise_and(align_hand, align_hand, mask = skinMask)

            depth_pixel = [(int(left)+int(right))/2, (int(top)+int(bottom))/2]
            dist = im_depth[depth_pixel[1],depth_pixel[0]]*depth_scale
            print dist
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dist)
            print depth_point
            hand_mark.counter = 0
            t = rospy.get_time()
            hand_mark.color = [1,0,0,1]
            m = hand_mark.marker(points= [(depth_point[0], depth_point[1], depth_point[2])])
            
            # rospy.loginfo(m)
            pub.publish(m)
            # rate.sleep()
             
            # show the skin in the image along with the mask
            cv2.imshow("images", np.hstack([align_hand, skin]))
            #end skin

        # Calculate Frames per second (FPS)
        num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        start_time).total_seconds()
        fps = num_frames / elapsed_time

        #display window
        if (args.display > 0):
            # Display FPS on frame
            if (args.fps > 0):
                detector_utils.draw_fps_on_image(
                    "FPS : " + str(float(fps)), image_np)

            cv2.imshow('Single Threaded Detection', cv2.cvtColor(
                image_np, cv2.COLOR_RGB2BGR))
        else:
            print("frames processed: ",  num_frames,
                  "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))

        if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break
#            

if __name__ == '__main__':
    main()
    print("Close Hand Detection Publisher")
    sess.close()

# Reference
# x
# https://answers.ros.org/question/312166/how-to-subcribe-2-image-topics-with-1-node-in-same-time/
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/