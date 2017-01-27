#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class WallApproach(object):
    def __init__(self):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.actual_distance = -1.0
        self.target = rospy.get_param('~target_distance')
        self.k = rospy.get_param('~k')

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.actual_distance = msg.ranges[0]

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.actual_distance != -1.0:
                error = self.actual_distance - self.target
                self.pub.publish(Twist(linear=Vector3(x=error*self.k)))
            r.sleep()

if __name__ == '__main__':
    node = WallApproach()
    node.run()