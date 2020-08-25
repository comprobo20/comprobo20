#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

class ReceiveMessageNode(object):
    def __init__(self):
        rospy.init_node('receive_message_node')
        rospy.Subscriber('/my_point', PointStamped, self.process_point)

    def process_point(self, m):
        print(m)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveMessageNode()
    node.run()
