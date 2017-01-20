#!/usr/bin/env python

""" This script is our first ROS node.  We'll receive some messages """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

def process_point(m):
    """ Process the point data from my topic """
    print m

rospy.init_node('test_receive_message')
rospy.Subscriber('/my_topic', PointStamped, process_point)

r = rospy.Rate(2)
while not rospy.is_shutdown():
    r.sleep()

print "Node is finished"