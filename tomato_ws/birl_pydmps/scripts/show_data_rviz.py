#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Pose
import ipdb
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import util
import random
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)

def main():
    rospy.init_node('test', anonymous=True)
    # raw_data = np.load('record_demo.npy')
    # raw_data = np.load('./demonstrations/record_demo_1_.npy')
    raw_data = np.load('/home/hayashi/worksp/tomato_ws/src/tomato_ws/handtracking/src/handtracking/hand_pub.npy')
    print(raw_data,raw_data.shape)
    # ipdb.set_trace()
    filtered_data = gaussian_filter1d(raw_data.T, sigma=15).T
    # filtered_data = raw_data
    filtered_data = util.filter_static_points(filtered_data)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    rospy.sleep(0.5)
    rgba_tuple = [random.uniform(0.5, 1), random.uniform(0.5, 1), random.uniform(0.5, 1), 1]
    display('publish visualization marker')
    print(filtered_data,raw_data.shape)

    #convert to world
    for idx, point in enumerate(filtered_data):
        # ipdb.set_trace()
        now_pose = Pose()
        now_pose.position.x = point[0]
        now_pose.position.y = point[1]
        now_pose.position.z = point[2]
        now_pose.orientation.x = point[3]
        now_pose.orientation.y = point[4]
        now_pose.orientation.z = point[5]
        now_pose.orientation.w = point[6]
        # util.send_traj_point_marker(marker_pub=marker_pub,frameid="/camera_color_optical_frame", pose=now_pose, id=idx, rgba_tuple=rgba_tuple)


        pos = now_pose.position
        ori = now_pose.orientation
        mat = np.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z,ori.w)))
        transform_mat = np.dot(translation_matrix((0.077, 0.035, 0.188)), quaternion_matrix((-0.500, 0.500, -0.500, 0.500)))
        new_mat = np.dot(transform_mat,mat)

        new_trans = translation_from_matrix(new_mat)
        new_quat = quaternion_from_matrix(new_mat)   
        
        new_pose = Pose()
        new_pose.position.x = new_trans[0]
        new_pose.position.y = new_trans[1]
        new_pose.position.z = new_trans[2]
        new_pose.orientation.x = new_quat[0]
        new_pose.orientation.y = new_quat[1]
        new_pose.orientation.z = new_quat[2]
        new_pose.orientation.w = new_quat[3]
        util.send_traj_point_marker(marker_pub=marker_pub,frameid="/world", pose=new_pose, id=idx+300, rgba_tuple=rgba_tuple)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass