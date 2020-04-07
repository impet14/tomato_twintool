#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_robot_state', anonymous=True)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander('manipulator')
reference_frame = 'iiwa_link_ee'
# reference_frame = 'world'
group.set_pose_reference_frame(reference_frame)

print "============ Printing robot state"
print group.get_current_pose()
print "============"

# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)
