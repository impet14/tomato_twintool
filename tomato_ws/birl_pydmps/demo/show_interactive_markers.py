#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import ipdb
from scipy.ndimage.filters import gaussian_filter1d
import util

def main():
    rospy.init_node('test', anonymous=True)
    csv_path = '/home/shuangda/ros/kuka_ws/src/birl_pydmps/demo/hand_marker.csv'
    data_csv = pd.read_csv(csv_path)
    raw_data = data_csv.values[:, 24:27].astype(float)
    filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    step_amount = 15
    for idx, point in enumerate(filtered_data):
        now_pose = Pose()
        now_pose.position.x = point[0]
        now_pose.position.y = point[1]
        now_pose.position.z = point[2]

        # visualize the pose in rviz using marker
        alpha = float(idx)/step_amount*0.5+0.5 
        rgba_tuple = (0, 0.5, 0.5, alpha)
        ipdb.set_trace()
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=count, rgba_tuple=rgba_tuple)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass