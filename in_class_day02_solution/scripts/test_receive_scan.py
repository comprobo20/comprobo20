#!/usr/bin/env python

""" This script is our first ROS node.  We'll receive some scan data """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
#from geometry_msgs.msg import Point
import rospy

def process_scan(m):
    """ Print out the distance to the obstacle immediately ahead
        of the robot """
    print m.ranges[0]

rospy.init_node('test_receive_scan')
rospy.Subscriber('/scan', LaserScan, process_scan)

r = rospy.Rate(2)
while not rospy.is_shutdown():
    r.sleep()

print "Node is finished"