#!/usr/bin/env python

import rospy
import rosparam
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('State_Publisher')

joints = ["twintool_joint1", "twintool_joint2_1"]

start_ID = 1

# j0_init = rosparam.get_param("/twintool/joint0_controller/motor/init")
j0_init = 2047
offset = [j0_init]
print('L: ', offset)
MX_list = [1]


def process(msg):
    joint_states = JointState()
    joint_states.header.stamp = rospy.Time.now()

    for x in msg.motor_states:
        # print ('x ', x)
        if (x.id in MX_list):
            resolution = 4095
            deg_range = 360.0

            if (resolution == 0):
                print(
                    'please check the offset value of pkg:dynamixelmotor node:state_publisher.py')
                print('Probably offset value inside code is wrong')
                break
            joint_states.name.append(joints[x.id-start_ID])
            joint_states.position.append(
                (x.position-offset[x.id-start_ID])*(deg_range/resolution)*(pi/180))
            joint_states.velocity.append(0.0)
            joint_states.effort.append(0.0)

    # fixed angle softrobotics
    joint_states.name.append(joints[1])
    joint_states.position.append(0.0)  # in radian
    joint_states.velocity.append(0.0)
    joint_states.effort.append(0.0)
    pub.publish(joint_states)


# Subscriber for raw feedback from dynamixel motor. Position of the motor will be in the range of (0,1023).
sub = rospy.Subscriber(
    '/twintool/motor_states/conbe_L_port', MotorStateList, process)
pub = rospy.Publisher(
	'/twintool/joint_states', JointState, queue_size=10)

rospy.spin()
