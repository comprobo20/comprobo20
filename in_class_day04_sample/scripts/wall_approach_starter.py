#!/usr/bin/env python3

"""
Maintain a target distance from the wall using the laser scan measurement in
front to estimate distance.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallApproach(object):
    """ Wraps the basic structure of the node """
    def __init__(self):
        rospy.init_node('wall_approacher')
        self.distance_to_wall = None
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process_scan(self, msg):
        """ Populate the attribute distance_to_wall if we have new data from
            the laser scanner right in front of us.
            Remember that missing data is reported as a value of 0 in ranges """
        pass

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.distance_to_wall is not None:
                # compute error for proportional control, create message,
                # and publish
                m = Twist()
                self.pub.publish(m)
            r.sleep()

if __name__ == "__main__":
    node = WallApproach()
    node.run()
