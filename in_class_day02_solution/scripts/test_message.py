#!/usr/bin/env python

""" This script is our first ROS node.  We'll publish some messages """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
#from geometry_msgs.msg import Point
import rospy

rospy.init_node('test_message')
loop_count = 0

publisher = rospy.Publisher('/my_topic', PointStamped, queue_size=10)

r = rospy.Rate(2)
while not rospy.is_shutdown():
    print "looping", loop_count
    loop_count += 1
    my_header = Header(loop_count, rospy.Time.now(), "odom")
    my_point = Point(x=3.2, y=5.4)
    my_point_stamped = PointStamped(point=my_point, header=my_header)
    publisher.publish(my_point_stamped)
    r.sleep()

print "Node is finished"