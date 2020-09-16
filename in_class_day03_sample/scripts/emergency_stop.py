#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3

class EmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        self.desired_velocity = 0.3
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)

    def process_bump(self, msg):
        if msg.data[0] == 1 or msg.data[1] == 1 or msg.data[2] == 1 or msg.data[3] == 1:
            self.desired_velocity = 0.0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity)))
            r.sleep()

if __name__ == '__main__':
    estop = EmergencyStopNode()
    estop.run()
