#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rospy

class RawVelRelayNode(object):
    def __init__(self):
        rospy.init_node('raw_vel_relay')
        rospy.Subscriber('raw_vel', Float32MultiArray, self.raw_vel_received)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # it is less important that this wheel base is accurate for the simulated robot but that it is true to the real robot
        self.real_robot_wheel_base = 0.24

    def raw_vel_received(self, msg):
        if len(msg.data) == 2:
            vL, vR = msg.data[0], msg.data[1]
            msg = Twist()
            msg.linear.x = (vL + vR)/2;
            msg.angular.z = (vR - vL)/self.real_robot_wheel_base;
            self.cmd_vel_pub.publish(msg)

    def spin(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = RawVelRelayNode()
    node.spin()
