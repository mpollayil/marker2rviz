#! /usr/bin/env python3

from tf import TransformBroadcaster
import rospy
from rospy import Time 
from geometry_msgs.msg import Pose

translation = (1, 2, 3)
rotation = (0.0, 0.0, 0.0, 1.0)

def callback(data):

    global translation
    global orientation

    translation = (data.position.x, data.position.y, data.position.z)
    orientation = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)


def main():
    
    global translation
    global orientation

    rospy.init_node('frame_broadcaster')
    
    b = TransformBroadcaster()
    rospy.Subscriber("marker_pose", Pose, callback)
    
    rate = rospy.Rate(1000)  # 1 khz

    rospy.loginfo("Managing marker_frame according to pose published to /marker_pose topic")
    
    while not rospy.is_shutdown():
        b.sendTransform(translation, rotation, Time.now(), 'marker_frame', 'base')
        rate.sleep()
    


if __name__ == '__main__':
    main()