#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

class TestMessageNode(object):
    """ This node publishes a message at 2 Hz """
    def __init__(self):
        rospy.init_node("test_node")
        self.pub = rospy.Publisher('/my_point', PointStamped, queue_size=10)

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")
            my_point = Point(x=1.0, y=2.0)
            m = PointStamped(header=my_header, point=my_point)
            self.pub.publish(m)
            r.sleep()

if __name__ == '__main__':
    node = TestMessageNode()
    node.run()
