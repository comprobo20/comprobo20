#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

def process_point(m):
    print(m)

rospy.init_node('receive_message_node')
rospy.Subscriber('/my_point', PointStamped, process_point)

rospy.spin()
