#!/usr/bin/env python3

"""
Maintain a target distance from the wall using the laser scan measurement in
front to estimate distance.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from dynamic_reconfigure.server import Server
from in_class_day04_sample.cfg import WallApproachConfig


class WallApproach(object):
    """ Wraps the basic structure of the node """
    def __init__(self):
        rospy.init_node('wall_approacher')
        self.target_distance = rospy.get_param('~target_distance', 1.0)
        self.k = rospy.get_param('~k', 0.5)

        srv = Server(WallApproachConfig, self.callback)

        self.distance_to_wall = None
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def callback(self, config, level):
        self.k = config.k
        self.target_distance = config.target_distance
        return config

    def process_scan(self, msg):
        """ Populate the attribute distance_to_wall if we have new data from
            the laser scanner right in front of us """
        if msg.ranges[0] > 0.0:
            self.distance_to_wall = msg.ranges[0]
            print(self.distance_to_wall)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.distance_to_wall is not None:
                # compute error for proportional control
                error = self.distance_to_wall - self.target_distance
                m = Twist()
                m.linear.x = self.k*error
                self.pub.publish(m)
            r.sleep()

if __name__ == "__main__":
    node = WallApproach()
    node.run()
