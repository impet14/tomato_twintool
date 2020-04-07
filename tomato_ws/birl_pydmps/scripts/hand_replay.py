#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Pose
# import ipdb
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import util
import random
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)


def arraytopose(pose_array):
    pose_list = []
    # pose_goal = geometry_msgs.msg.Pose()
    for i in range(pose_array.shape[0]):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = pose_array[i - 1][0]
        pose.position.y = pose_array[i - 1][1]
        pose.position.z = pose_array[i - 1][2]
        pose.orientation.x = pose_array[i - 1][3]
        pose.orientation.y = pose_array[i - 1][4]
        pose.orientation.z = pose_array[i - 1][5]
        pose.orientation.w = pose_array[i - 1][6]
        pose_list.append(pose)

    print(pose_list)
    return pose_list


def main():

    # moveit ros
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_move_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')
    scene = moveit_commander.PlanningSceneInterface()
    planning_frame = group.get_planning_frame()
    
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    print(group.get_current_pose())
    joint_goal = group.get_current_joint_values()
    joint_goal = [0, 0, 0, 0, 0, 0, 0]
    group.go(joint_goal, wait=True)
    group.stop()

    raw_data = np.load(
        '/home/hayashi/worksp/tomato_ws/src/tomato_ws/handtracking/src/handtracking/hand_pub.npy')
    print(raw_data, raw_data.shape)

    filtered_data = gaussian_filter1d(raw_data.T, sigma=15).T
    filtered_data = util.filter_static_points(filtered_data)
    marker_pub = rospy.Publisher(
        "/visualization_marker", Marker, queue_size=100)
    rospy.sleep(0.5)
    rgba_tuple = [random.uniform(0.5, 1), random.uniform(
        0.5, 1), random.uniform(0.5, 1), 1]
    display('publish visualization marker')
    print(filtered_data, raw_data.shape)

    # convert to world
    waypoints = []
    for idx, point in enumerate(filtered_data):
        # ipdb.set_trace()
        now_pose = Pose()
        now_pose.position.x = point[0]
        now_pose.position.y = point[1]
        now_pose.position.z = point[2]
        
        pos = now_pose.position
        ori = now_pose.orientation
        mat = np.dot(translation_matrix((pos.x, pos.y, pos.z)),
                     quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))
        transform_mat = np.dot(translation_matrix(
            (0.077, 0.035, 0.188)), quaternion_matrix((-0.500, 0.500, -0.500, 0.500)))
        new_mat = np.dot(transform_mat, mat)

        new_trans = translation_from_matrix(new_mat)
        new_quat = quaternion_from_matrix(new_mat)

        new_pose = Pose()
        new_pose.position.x = new_trans[0]
        new_pose.position.y = new_trans[1]
        new_pose.position.z = new_trans[2]

        new_pose.orientation.x = 2.6944e-5
        new_pose.orientation.y = 0.70713
        new_pose.orientation.z = 2.5173e-5
        new_pose.orientation.w = 0.70708

        util.send_traj_point_marker(
            marker_pub=marker_pub, frameid="/world", pose=new_pose, id=idx+300, rgba_tuple=rgba_tuple)

        waypoints.append(copy.deepcopy(new_pose))

    group.set_max_velocity_scaling_factor(0.01)
    for i in range (3):
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.005,       # eef_step
            0.0)         # jump_threshold

    # with open ('plan_handreplay.txt', 'w') as f: f.write (str(plan))
    group.execute(plan, wait=True)
    print group.get_current_pose()
    group.stop()
    group.clear_pose_targets()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
