import rospy
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
import ipdb
# ipdb.set_trace()
def main():
    rospy.init_node('dummy_marker', anonymous=True)
    rospy.loginfo("Marker Dummy Start!")
    marker1_pub=rospy.Publisher('/object/box', Marker,queue_size=1)
    marker1 = MarkerGenerator()
    marker1.type = Marker.SPHERE_LIST
    marker1.scale = [.04]*3
    marker1.frame_id = '/camera_color_optical_frame'
    marker1.color = [1,0,0,1]

    marker2_pub=rospy.Publisher('/object/karaage', Marker,queue_size=1)
    marker2 = MarkerGenerator()
    marker2.type = Marker.SPHERE_LIST
    marker2.scale = [.04]*3
    marker2.frame_id = '/camera_color_optical_frame'
    marker2.color = [0,0,1,1]

    marker3_pub=rospy.Publisher('/object/onigiri', Marker,queue_size=1)
    marker3 = MarkerGenerator()
    marker3.type = Marker.SPHERE_LIST
    marker3.scale = [.04]*3
    marker3.frame_id = '/camera_color_optical_frame'
    marker3.color = [0,0,1,1]

    marker4_pub=rospy.Publisher('/object/chip', Marker,queue_size=1)
    marker4 = MarkerGenerator()
    marker4.type = Marker.SPHERE_LIST
    marker4.scale = [.04]*3
    marker4.frame_id = '/camera_color_optical_frame'
    marker4.color = [0,0,1,1]

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        marker1.counter = 0
        ipdb.set_trace()
        marker1_pub.publish(marker1.marker(points= [(0.25, 0.12, 0.35)], position = (0.25, 0.12, 0.35),orientation=(0,0,0,1)))
        
        marker2.counter = 0
        marker2_pub.publish(marker2.marker(position= (-0.1, 0.10, 0.38),orientation=(0,0,0,1)))
        
        marker3.counter = 0
        marker3_pub.publish(marker3.marker(position= (-0.15, 0.11, 0.38),orientation=(0,0,0,1)))
        
        marker4.counter = 0
        marker4_pub.publish(marker4.marker(position= (-0.2, 0.125, 0.38),orientation=(0,0,0,1)))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass