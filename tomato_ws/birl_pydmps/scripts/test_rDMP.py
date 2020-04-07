

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

joint_1 = [2.83280979458, 0.729731043086, -0.383054208306, 1.11708416386, -0.493804670028, 0.495311417526, -2.0955462426]

joint_2 = [2.93213695947, 0.263106197503, -1.28026775814, 1.35847770065, -0.262342999048, 1.29072712375, -1.58758119812]

joint_3 = [2.47288612753, -0.364583623266, -1.33046895051, 0.607562665087, 0.521168305515, 0.768950130155, -1.55000834251]

def main():
    rospy.init_node('move_test', anonymous=True)

    # moveit
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('manipulator')

    plan = util.fK_point_calculate(group,JointAngle=joint_1)
    util.execute_plan(group,plan)    

    plan = util.fK_point_calculate(group,JointAngle=joint_2)
    util.execute_plan(group,plan)    
    
    plan = util.fK_point_calculate(group,JointAngle=joint_3)
    util.execute_plan(group,plan)    

    plan = util.fK_point_calculate(group,JointAngle=joint_1)
    util.execute_plan(group,plan)   

    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass