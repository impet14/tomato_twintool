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

def main():
    rospy.init_node('test', anonymous=True)
    # raw_data = np.load('record_demo.npy')
    raw_data = np.load('./demonstrations/record_demo_1_.npy')
    # ipdb.set_trace()
    filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T
    # filtered_data = util.filter_static_points(filtered_data,0.03)
    marker_pub = rospy.Publisher("/iiwa/visualization_marker", Marker, queue_size=100)
    rospy.sleep(0.5)
    rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]
    
    now_pose = Pose() # buffer pose

    #calculate mean point
    mean_point = [0.0,0.0,0.0] #x,y,z
    for idx, point in enumerate(filtered_data):
        mean_point[0] += point[0]
        mean_point[1] += point[1]
        mean_point[2] += point[2]
    for i in range(len(mean_point)):
        mean_point[i] = mean_point[i]/len(filtered_data)

    #show meanpoint rviz with marker
    now_pose.position.x =mean_point[0]
    now_pose.position.y =mean_point[1]
    now_pose.position.z =mean_point[2]
    util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=1000, rgba_tuple=[0.1,0.1,0.1,1])
    
    # calculate offset traj on zero center point
    filtered_data_zero = filtered_data
    for idx,point in enumerate(filtered_data):
        filtered_data_zero[idx][:3] = point[:3]-mean_point

    #display offset traj on zero center point
    for idx, point in enumerate(filtered_data_zero):
        # ipdb.set_trace()
        now_pose.position.x = point[0] #+mean_point[0]
        now_pose.position.y = point[1] #+mean_point[1]
        now_pose.position.z = point[2] #+mean_point[2]
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=idx, rgba_tuple=rgba_tuple)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass