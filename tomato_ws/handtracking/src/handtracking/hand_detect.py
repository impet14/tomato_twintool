#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from random import randint

# define the random_number Publisher
def random_number_publisher():
    rospy.init_node('random_number')
    pub=rospy.Publisher('rand_no', Int32, queue_size=10)
    rate= rospy.Rate(2)
# //generate a random number at every 2 seconds
    while not rospy.is_shutdown():
        random_msg=randint(0,5000)
        rospy.loginfo(random_msg)
        pub.publish(random_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        random_number_publisher()
    except rospy.ROSInterruptException:
        pass