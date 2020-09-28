#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from simple_filter.msg import LaserSimple, OdometrySimple
from nav_msgs.msg import Odometry

class NeatoBridge(object):
    def __init__(self):
        rospy.init_node('neato_bridge')
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/simple_scan', LaserSimple, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.pub_odom = rospy.Publisher('/simple_odom', OdometrySimple, queue_size=10)
        self.last_odom = None

    def process_scan(self, msg):
        self.pub.publish(LaserSimple(north_laser=msg.ranges[0],
                                     south_laser=msg.ranges[180],
                                     east_laser=msg.ranges[270],
                                     west_laser=msg.ranges[90]))

    def process_odom(self,msg):
        if (self.last_odom == None or
            abs(self.last_odom[0] - msg.pose.pose.position.x)>0.1 or
            abs(self.last_odom[1] - msg.pose.pose.position.y)>0.1):
            self.pub_odom.publish(OdometrySimple(south_to_north_position=msg.pose.pose.position.x,
                                                 west_to_east_position=msg.pose.pose.position.y))
            self.last_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = NeatoBridge()
    node.run()
