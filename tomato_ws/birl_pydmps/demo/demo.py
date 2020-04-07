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

csv_path = '/home/shuangda/ros/kuka_ws/src/birl_pydmps/demo/hand_marker.csv'
data_csv = pd.read_csv(csv_path)
raw_data = data_csv.values[:, 24:27].astype(float)
filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T

dmp = DMPs_rhythmic(n_dmps=3, n_bfs=200)
dmp.imitate_path(y_des=filtered_data.T)
# dmp.y0 = np.array([ 0.00751207,  0.00487747,  0.41364701])
# dmp.goal = np.array([-0.01121142, -0.01731688,  0.41768865])
y_track, dy_track, ddy_track = dmp.rollout()

dmp = DMPs_discrete(n_dmps=3, n_bfs=200)
dmp.imitate_path(y_des=filtered_data.T)
y_track_d, dy_track_d, ddy_track_d = dmp.rollout()

# # plot raw data
# fig = plt.figure(1,figsize=(10,8))
# ax = fig.gca(projection='3d')
# ax.plot(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], 'b', linewidth=3, linestyle='-')


# fig = plt.figure(2,figsize=(10,8))
# ax = fig.gca(projection='3d')
# ax.scatter(filtered_data[0, 0],filtered_data[0, 1],filtered_data[0, 2], color= "r",s=100)
# ax.scatter(filtered_data[-1, 0],filtered_data[-1, 1],filtered_data[-1, 2], color= "b",s=100)
# ax.plot(filtered_data[:, 0], filtered_data[:, 1], filtered_data[:, 2], linewidth=3, linestyle='-')

# ax.scatter(y_track[0, 0],y_track[0, 1],y_track[0, 2], color= "g",s=100)
# ax.scatter(y_track[-1, 0],y_track[-1, 1],y_track[-1, 2], color= "black",s=100)
# ax.plot(y_track[:, 0], y_track[:, 1], y_track[:, 2], linewidth=3, linestyle='--')

# fig = plt.figure(3,figsize=(10,8))
# ax = fig.gca(projection='3d')
# ax.plot(y_track_d[:, 0], y_track_d[:, 1], y_track_d[:, 2], linewidth=3, linestyle='--')

# plt.show()

# moveit
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_move_test', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('manipulator')
reference_frame = 'iiwa_link_ee'
group.set_pose_reference_frame(reference_frame)
end_effector_link = group.get_end_effector_link()

target_pose = Pose() 
target_pose.position.x = y_track[0,0]
target_pose.position.y = y_track[0,1]
target_pose.position.z = y_track[0,2]
target_pose.orientation.x = 0
target_pose.orientation.y = 0
target_pose.orientation.z = 0
target_pose.orientation.w = 1

group.set_start_state_to_current_state()
group.set_pose_target(target_pose, end_effector_link)
plan = group.plan()
group.execute(plan,wait=True)
rospy.sleep(1)

# waypoints = []  
# waypoints.append(group.get_current_pose().pose)

# for idx in xrange(y_track.shape[0]):
#     wpose = Pose()
#     wpose.position.x = y_track[idx,0]
#     wpose.position.y = y_track[idx,1]
#     wpose.position.z = y_track[idx,2]
#     wpose.orientation.x = 0
#     wpose.orientation.y = 0
#     wpose.orientation.z = 0
#     wpose.orientation.w = 1
#     waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = group.compute_cartesian_path(
#                             waypoints,   # waypoints to follow
#                             0.01,        # eef_step
#                             0.0)         # jump_threshold
ipdb.set_trace()

group.execute(plan,wait=True)
# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)



