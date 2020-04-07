#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Float64,Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped,Pose
import ipdb
from scipy.ndimage.filters import gaussian_filter1d
from birl_pydmps import util
import tf
from math import pi

def gripper(state=False):
  msg = Bool()
  pub = rospy.Publisher('/twintool/gripper', Bool, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  if state:
    msg.data = True
    pub.publish(msg);pub.publish(msg);

  else:
    msg.data = False
    pub.publish(msg);pub.publish(msg);

def gripper_profile(profile): #0 all low, 1 grip, 2 blow air
  msg = Int8()
  pub = rospy.Publisher('/twintool/gripper_profile', Int8, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  msg.data = profile
  pub.publish(msg);pub.publish(msg);

def vacuum(state): #0 all low, 1 grip, 2 blow air
  msg = Int8()
  pub = rospy.Publisher('/twintool/vacuum', Int8, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  msg.data = state
  pub.publish(msg);pub.publish(msg);
 

def twintool_change_ee(state=False):  # true : gripper , false : vacuum
  msg = Float64()
  pub = rospy.Publisher('/twintool/joint1_controller/command', Float64, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  if state:
    msg.data = 3.1415
    pub.publish(msg);pub.publish(msg);

  else:
    msg.data = 0.0
    pub.publish(msg);pub.publish(msg);

def main():
    rospy.init_node('cartesian_move_test', anonymous=True)

    #set twintool for gripper init
    print ('initial twintool joint')
    twintool_change_ee(True)
    gripper(False)

    # return 
    # moveit ros
    moveit_commander.roscpp_initialize(sys.argv)
    
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

    # init pose
    joint_command = [0,0,0,0,0,-pi/2,0]
    # util.speed_set(group,0.5)
    plan = util.fK_point_calculate(group,JointAngle=joint_command)
    util.execute_plan(group,plan)
    rospy.sleep(2.0)

    marker1_topic = "/object/box"
    marker2_topic = "/object/karaage"
    marker3_topic = "/object/onigiri"
    marker4_topic = "/object/chip"

    print("wait makers message")
    marker1_msg = rospy.wait_for_message(marker1_topic, Marker,timeout=3)
    marker2_msg = rospy.wait_for_message(marker2_topic, Marker,timeout=3)

    # point transform to world
    print("wait for listener")
    listener = tf.TransformListener()
    listener.waitForTransform("/world", "/camera_color_optical_frame", rospy.Time(0),rospy.Duration(4.0))
    # ipdb.set_trace()
    transformed_point = []
    transformed_point.append(copy.deepcopy(PointStamped()))
    transformed_point[0].header = marker1_msg.header
    # ipdb.set_trace()
    transformed_point[0].point = marker1_msg.pose.position
    transformed_point[0].header.stamp = rospy.Time(0)
    transformed_point[0]= listener.transformPoint("/world", transformed_point[0])
    transformed_point[0].point.z += 0.02
    
    transformed_point.append(copy.deepcopy(PointStamped()))
    transformed_point[1].header = marker2_msg.header
    transformed_point[1].point = marker2_msg.pose.position
    transformed_point[1].header.stamp = rospy.Time(0)
    transformed_point[1]= listener.transformPoint("/world", transformed_point[1])
    transformed_point[1].point.x -= 0.03
    transformed_point[1].point.y += 0.015
    transformed_point[1].point.z -= 0.04

    print (transformed_point)
    print('len marker', len(transformed_point))
    for i in range (len(transformed_point)) : 
        print('i ', i )
        waypoints = []
        #prepick
        wpose = Pose()
        wpose.position.x = transformed_point[1].point.x
        wpose.position.y = transformed_point[1].point.y
        wpose.position.z = transformed_point[1].point.z + 0.1
        wpose.orientation.x = 2.6944e-5
        wpose.orientation.y = 0.70713
        wpose.orientation.z = 2.5173e-5
        wpose.orientation.w = 0.70708
        waypoints.append(copy.deepcopy(wpose))

        #pick
        wpose.position.z = transformed_point[1].point.z
        wpose.orientation.x = 2.6944e-5
        wpose.orientation.y = 0.70713
        wpose.orientation.z = 2.5173e-5
        wpose.orientation.w = 0.70708
        waypoints.append(copy.deepcopy(wpose))
        
        # group.set_max_velocity_scaling_factor(0.01)
        for i in range (6):
            (plan, fraction) = group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,       # eef_step
                0.0)         # jump_threshold
        group.execute(plan, wait=True)
        gripper(True)
        rospy.sleep(1.5)

        waypoints = []
        wpose.position.z = wpose.position.z + 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = transformed_point[0].point.x
        wpose.position.y = transformed_point[0].point.y
        wpose.position.z = transformed_point[0].point.z + 0.1
        wpose.orientation.x = 2.6944e-5
        wpose.orientation.y = 0.70713
        wpose.orientation.z = 2.5173e-5 
        wpose.orientation.w = 0.70708
        waypoints.append(copy.deepcopy(wpose))

        # place
        wpose.position.x = transformed_point[0].point.x
        wpose.position.y = transformed_point[0].point.y
        wpose.position.z = transformed_point[0].point.z
        wpose.orientation.x = 2.6944e-5
        wpose.orientation.y = 0.70713
        wpose.orientation.z = 2.5173e-5 
        wpose.orientation.w = 0.70708
        waypoints.append(copy.deepcopy(wpose))

        group.set_max_velocity_scaling_factor(0.01)
        for i in range (6):
            (plan, fraction) = group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,       # eef_step
                0.0)         # jump_threshold
        group.execute(plan, wait=True)

        gripper(False)
        rospy.sleep(1.5)

        waypoints = []
        wpose.position.x = transformed_point[0].point.x
        wpose.position.y = transformed_point[0].point.y
        wpose.position.z = transformed_point[0].point.z + 0.1
        wpose.orientation.x = 2.6944e-5
        wpose.orientation.y = 0.70713
        wpose.orientation.z = 2.5173e-5 
        wpose.orientation.w = 0.70708
        waypoints.append(copy.deepcopy(wpose))

        group.set_max_velocity_scaling_factor(0.01)
        for i in range (5):
            (plan, fraction) = group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,       # eef_step
                0.0)         # jump_threshold
        group.execute(plan, wait=True)

        break



    # init pose
    joint_command = [0,0,0,0,0,-pi/2,0.0]
    util.speed_set(group,1)
    plan = util.fK_point_calculate(group,JointAngle=joint_command)
    util.execute_plan(group,plan)  

    group.stop()
    group.clear_pose_targets()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass