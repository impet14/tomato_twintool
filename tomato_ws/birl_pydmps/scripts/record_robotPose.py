#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np 
import ipdb
from birl_pydmps import util


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_robot_state', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('conbe')
    # reference_frame = 'iiwa_link_ee'
    reference_frame = 'world'
    group.set_pose_reference_frame(reference_frame)

    pose_list = []
    while not rospy.is_shutdown():
        pose_ =  group.get_current_pose().pose
        mat = [pose_.position.x, pose_.position.y, pose_.position.z,
               pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w,]
        print mat
        pose_list.append(mat)
    
    pose_list = util.filter_static_points(np.array(pose_list))
    # ipdb.set_trace()
    np.save("record_demo",pose_list)
    print "Save"
    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass