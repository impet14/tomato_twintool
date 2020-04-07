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
from geometry_msgs.msg import PointStamped,Pose
import ipdb
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import util
import random
import tf
# import pathlib

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

    print (pose_list)
    return pose_list

    # for item in list:
    #     print item.position.z
# def cb_hand(msg):
#     print ("cb hand marker")
#     print (msg)
#     sub_hand_marker.unregister()

# global sub_hand_marker
def main():
    
    # moveit ros
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_move_test', anonymous=True)

    hand_marker_node = "/hand_marker"
    print("wait /hand_maker message")
    hand_msg = rospy.wait_for_message(hand_marker_node, Marker)
    hand_point = hand_msg.points
    # ipdb.set_trace()

    marker_pub = rospy.Publisher("/iiwa/visualization_marker", Marker, queue_size=100)
    rospy.sleep(0.5)
    rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]

    listener = tf.TransformListener()
    listener.waitForTransform("/iiwa_link_0", "/camera_color_optical_frame", rospy.Time(0),rospy.Duration(4.0))

    laser_point=PointStamped()
    laser_point.header.frame_id = "camera_color_optical_frame"
    laser_point.header.stamp =rospy.Time(0)
    laser_point.point.x=hand_point[0].x
    laser_point.point.y=hand_point[0].y
    laser_point.point.z=hand_point[0].z
    p=listener.transformPoint("iiwa_link_0",laser_point)
    print (hand_point)
    print (p)
    # return

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names() 

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    #motion path
    # raw_data = np.load('record_demo.npy')
    raw_data = np.load('./demonstrations/record_demo_1.npy')
    filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T
    with open ('waypoint.txt', 'w') as f: f.write (str(filtered_data))

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
    # util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=1000, rgba_tuple=[0.1,0.1,0.1,1])
    util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=1000, rgba_tuple=[0.1,0.1,0.1,1])
    
    # calculate offset traj on zero center point
    filtered_data_zero = filtered_data
    for idx,point in enumerate(filtered_data):
        filtered_data_zero[idx][:3] = point[:3]-mean_point

    # filtered_data = util.filter_static_points(filtered_data,0.03)
    # util.plot_3d_demo(filtered_data)
    
    # joint_command = [0.11133063584566116, 0.5712737441062927,
    # -0.09774867445230484, -1.7133415937423706, -0.036780450493097305, -0.5433750152587891, 0.17699939012527466]

    # util.speed_set(group,0.01)
    # plan = util.fK_point_calculate(group,JointAngle=joint_command)
    # util.execute_plan(group,plan) 

    # for point in filtered_data:
    #     plan = util.iK_point_calculate(group,point=point)
    #     util.execute_plan(group,plan)    
    # #     ipdb.set_trace()    
    # ipdb.set_trace()

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 2.08126774e-02
    pose_goal.orientation.y = 7.65273086e-01
    pose_goal.orientation.z = 4.29933739e-02
    pose_goal.orientation.w = 6.41930552e-01
    
    pose_goal.position.x = p.point.x
    pose_goal.position.y = p.point.y
    pose_goal.position.z = p.point.z+0.1
    # group.set_pose_target(pose_goal)

    # return
    ## Now, we call the planner to compute the plan and execute it.
    # plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    # group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    # group.clear_pose_targets()
    # return


    waypoints = []
    scale = 0.1
    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.3  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    #display offset traj on zero center point
    for idx, point in enumerate(filtered_data_zero):
        # ipdb.set_trace()
        now_pose.position.x = point[0] +p.point.x
        now_pose.position.y = point[1] +p.point.y
        now_pose.position.z = point[2] +p.point.z
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=idx, rgba_tuple=rgba_tuple)

    # waypoints.append(copy.deepcopy(pose_goal))
    # pose_goal = geometry_msgs.msg.Pose()
    for i in range(filtered_data_zero.shape[0]):
        pose_goal.position.x = filtered_data_zero[i][0] +p.point.x
        pose_goal.position.y = filtered_data_zero[i][1] +p.point.y
        pose_goal.position.z = filtered_data_zero[i][2] +p.point.z +0.05
        pose_goal.orientation.x = filtered_data_zero[i][3]
        pose_goal.orientation.y = filtered_data_zero[i][4]
        pose_goal.orientation.z = filtered_data_zero[i][5]
        pose_goal.orientation.w = filtered_data_zero[i][6]
        waypoints.append(copy.deepcopy(pose_goal))

   
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    group.set_max_velocity_scaling_factor(0.01)
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    # (plan, fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
    
    group.execute(plan, wait=True)
    print group.get_current_pose()
    group.stop()
    group.clear_pose_targets()

    #original
    # plan = util.ik_cartesain_path(group, filtered_data[:])
    # # plan = util.set_trajectory_speed(plan, 0.02)
    # # plan = util.set_trajectory_speed(plan, 0.05)
    # # ipdb.set_trace()
    # with open ('plan.txt', 'w') as f: f.write (str(plan))
    # # util.execute_plan(group,plan)
    # group.execute(plan,wait=True)
    # group.stop()
    # group.clear_pose_targets()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass