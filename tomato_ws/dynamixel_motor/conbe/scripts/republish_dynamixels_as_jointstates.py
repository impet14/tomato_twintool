#! /usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState as dynJointState
from sensor_msgs.msg import JointState as sensJointState

# https://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html :
# Header header
# string name           # joint name
# int32[] motor_ids     # motor ids controlling this joint
# int32[] motor_temps   # motor temperatures, same order as motor_ids
#
# float64 goal_pos      # commanded position (in radians)
# float64 current_pos   # current joint position (in radians)
# float64 error         # error between commanded and current positions (in radians)
# float64 velocity      # current joint speed (in radians per second)
# float64 load          # current load
# bool is_moving        # is joint currently in motion
#
# https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html
# Header header
# string[] name
# float64[] position    # the position of the joint (rad or m)
# float64[] velocity    # the velocity of the joint (rad/s or m/s)
# float64[] effort      # the effort that is applied in the joint (Nm or N)


def convert(dynamixel_msg):
    """Convert a dynamixel_msgs/JointState to a sensor_msgs/JointState
    sensor_msgs/JointState consists of a list of joints but we publish one at a time"""
    sensor_msg = sensJointState(header=dynamixel_msg.header,
                                name=[dynamixel_msg.name],
                                position=[dynamixel_msg.current_pos],
                                velocity=[dynamixel_msg.velocity],
                                # TODO: verify the units of load and effort are the same
                                # I assume dynamixel load is also in Nm or N
                                effort=[dynamixel_msg.load],
                                )

    return sensor_msg


if __name__ == "__main__":
    rospy.init_node("republish_dynamixel_jointstates", log_level=rospy.INFO)

    joint_state_pub = rospy.Publisher(
        "joint_states", sensJointState, queue_size=1)

    dynamixel_joint_topic = rospy.Subscriber(
        "dynamixel_topic", dynJointState, lambda msg: joint_state_pub.publish(convert(msg)))

    rospy.spin()
