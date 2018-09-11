#!/usr/bin/env python

"""
Created on 29 July 2012
@author: Lisa Simpson
"""

from __future__ import print_function, division
import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3

class EmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = 0.3

    def process_bump(self, msg):
        if any((msg.leftFront, msg.leftSide, msg.rightFront, msg.rightSide)):
            self.desired_velocity = 0.0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity)))
            r.sleep()

if __name__ == '__main__':
    estop = EmergencyStopNode()
    estop.run()
