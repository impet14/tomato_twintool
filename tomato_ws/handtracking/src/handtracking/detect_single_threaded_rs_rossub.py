#!/usr/bin/env python
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
import rospy
from sensor_msgs.msg import Image

from utils import detector_utils as detector_utils
import tensorflow as tf
import datetime
import argparse
# from utils.detector_utils import WebcamVideoStream
import pyrealsense2 as rs
import numpy as np
# from open3d import *
# import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters

detection_graph, sess = detector_utils.load_inference_graph()
bridge = CvBridge()

def callback(color_data,depth_data):
    try:
        cv_image = bridge.imgmsg_to_cv2(color_data, "passthrough")
        cv_depth = bridge.imgmsg_to_cv2(depth_data, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    depth_image = np.asanyarray(cv_depth)

    # Render images
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Show the converted image
    cv2.imshow("Image Window", cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))
    cv2.imshow("Image Depth Window", depth_colormap)
    cv2.waitKey(3)  

if __name__ == '__main__':

    rospy.init_node('hand_tracking',anonymous=True)
    rospy.loginfo("Hello ROS!")

    #publishing hand marker 
    pub=rospy.Publisher('/hand_mark', Marker)
    gen = MarkerGenerator()
    rate= rospy.Rate(2)
    gen.type = Marker.SPHERE_LIST
    gen.scale = [.03]*3
    gen.frame_id = '/camera_color_optical_frame'
    # count = 0

    #subscribe realsense color and depth
    image_align_hand_color = "/camera/color/image_rect_color"
    image_align_hand_depth = "/camera/aligned_depth_to_color/image_raw"
    sub_image = message_filters.Subscriber(image_align_hand_color, Image)
    sub_depth_image = message_filters.Subscriber(image_align_hand_depth, Image)
    ts = message_filters.TimeSynchronizer([sub_image,sub_depth_image],10)
    ts.registerCallback(callback)
    
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
                        default=1, help='Display the detected images using OpenCV. This reduces FPS')
    parser.add_argument('-num-w', '--num-workers', dest='num_workers', type=int,
                        default=4, help='Number of workers.')
    parser.add_argument('-q-size', '--queue-size', dest='queue_size', type=int, 
                        default=5, help='Size of the queue.')
    args = parser.parse_args()

    #initial realsense camera
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)    
    # profile = pipeline.start(config)

    im_width, im_height =  (args.width, args.height) 
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    # depth_sensor = profile.get_device().first_depth_sensor()
    # # depth_scale = depth_sensor.get_depth_scale()
    # print "Depth Scale is: " , depth_scale

    #getting camera information for pc
    # profile_stream = profile.get_stream(rs.stream.depth)
    # intr = profile_stream.as_video_stream_profile().get_intrinsics()
    # print "ppx ppy width height are: " , intr.ppx, intr.ppy, intr.width,intr.height,intr.fx, intr.fy
    # print intr


    # depth_stream = profile.get_stream()
    #  clipping_distance_in_meters meters away
    # clipping_distance_in_meters = 0.6   #1 meter
    # clipping_distance = clipping_distance_in_meters / depth_scale

    # web camera source
    # cap = cv2.VideoCapture(args.video_sourcde)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    start_time = datetime.datetime.now()
    num_frames = 0
    # im_width, im_height = (cap.get(3), cap.get(4))
    # im_width, im_height = (args.width, args.height)
    # cap_params['im_width'], cap_params['im_height'] = video_capture.size()
    # max number of hands we want to detect/track

    
    # align_to = rs.stream.color
    # align = rs.align(align_to)

    num_hands_detect = 1

    #initial point cloud object from realsense
    # pc = rs.pointcloud()
    # points = rs.points()

    #skin filter 
    lower = np.array([0, 48, 80], dtype = "uint8")
    upper = np.array([20, 255, 255], dtype = "uint8")
    
    
    # while True with ros check
    while not rospy.is_shutdown():
        # print("hand detect")
        rospy.spin()

        rsframe = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(rsframe)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # color_rsframe = color_frame
        if not aligned_depth_frame or not color_frame:
            continue

        # Fetch color and depth frames
        # depth = aligned_depth_frame
        # points = pc.calculate(depth)
        # color = color_frame
        # pc.map_to(color)
        # points.export_to_ply("hand.ply", color)
        # print points

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 155
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        
        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        
        #########################align part###########################
        image_np = np.asanyarray(color_frame.get_data())

        # image_np = cv2.flip(image_np, 1)
        # start hand detection
        try:
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        except:
            print("Error converting to RGB")

        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', color_image)
        cv2.namedWindow('image_np', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('image_np', image_np)
        # actual detection
        boxes, scores = detector_utils.detect_objects(
            image_np, detection_graph, sess)

        # print boxes

        # draw bounding boxes
        detector_utils.draw_box_on_image(
            num_hands_detect, args.score_thresh, scores, boxes, im_width, im_height, image_np)

        if (scores[0] > args.score_thresh):
            (left, right, top, bottom) = (boxes[0][1] * im_width, boxes[0][3] * im_width,
                                          boxes[0][0] * im_height, boxes[0][2] * im_height)
            p1 = (int(left), int(top))
            p2 = (int(right), int(bottom))
            # print p1,p2,int(left),int(top),int(right),int(bottom)
            image_hand = image_np[int(top):int(bottom), int(left):int(right)]
            cv2.namedWindow("hand", cv2.WINDOW_NORMAL)
            cv2.imshow('hand', cv2.cvtColor(image_hand, cv2.COLOR_RGB2BGR))

            align_hand = bg_removed[int(top):int(bottom), int(left):int(right)]
            align_depth = depth_colormap[int(top):int(bottom),int(left):int(right)]
            align_hand_detect = np.hstack((align_hand, align_depth))
            cv2.namedWindow('align hand', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('align hand', align_hand)
            cv2.namedWindow('hand depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('hand depth', align_depth)

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
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_image[depth_pixel[1],depth_pixel[0]]*depth_scale)
            print depth_point
            gen.counter = 0
            t = rospy.get_time()
            gen.color = [1,0,0,1]
            m = gen.marker(points= [(depth_point[0], depth_point[1], depth_point[2])])
             
            # rospy.loginfo(m)
            pub.publish(m)
            rate.sleep()
            # print depth_intrin
            # print depth_image
            # print depth_image.shape
             
            # show the skin in the image along with the mask
            cv2.imshow("images", np.hstack([align_hand, skin]))
            #end skin

        # Calculate Frames per second (FPS)
        num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        start_time).total_seconds()
        fps = num_frames / elapsed_time

        if (args.display > 0):
            # Display FPS on frame
            if (args.fps > 0):
                detector_utils.draw_fps_on_image(
                    "FPS : " + str(float(fps)), image_np)

            cv2.imshow('Single Threaded Detection', cv2.cvtColor(
                image_np, cv2.COLOR_RGB2BGR))

            if cv2.waitKey(25) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break
        else:
            print("frames processed: ",  num_frames,
                  "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))
        # raw_input('Press enter to continue: ')