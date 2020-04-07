#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import DMPs_rhythmic
from birl_pydmps import DMPs_discrete
import ipdb
from birl_pydmps import util
from visualization_msgs.msg import Marker
import random 
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)


# csv_path = 'hand_marker_1.csv'
csv_path = 'hand.csv'
data_csv = pd.read_csv(csv_path)
raw_data = data_csv.values[60:-40, 24:27].astype(float)
display(raw_data)
filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T


rospy.init_node('robot_move_test', anonymous=True)
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
rospy.sleep(0.5)
rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]


for idx, point in enumerate(filtered_data):
    new_pose = Pose()
    new_pose.position.x = point[0]
    new_pose.position.y = point[1]
    new_pose.position.z = point[2]  
    util.send_traj_point_marker(marker_pub=marker_pub, pose=new_pose, id=idx, rgba_tuple=rgba_tuple)

ipdb.set_trace()
rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]
for idx, point in enumerate(filtered_data):
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2] 
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    pos = pose.position
    ori = pose.orientation
    mat = np.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z,ori.w)))
    transform_mat = np.dot(translation_matrix((0.017438, 0.110540, -0.003173)), quaternion_matrix((0,0,1,0)))
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
    util.send_traj_point_marker(marker_pub=marker_pub, pose=new_pose, id=idx+300, rgba_tuple=rgba_tuple)



