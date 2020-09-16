#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class DistanceEmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        self.desired_velocity = 0.3
        self.target_distance = 1.0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_scan(self, msg):
        if msg.ranges[0] != 0:
            if msg.ranges[0] < self.target_distance:
                self.desired_velocity = 0.0
            else:
                self.desired_velocity = 0.3

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity)))
            r.sleep()

if __name__ == '__main__':
    estop = DistanceEmergencyStopNode()
    estop.run()
