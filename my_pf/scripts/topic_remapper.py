#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class TopicRemapper(object):
    def __init__(self):
        rospy.init_node('topic_remapper')
        rospy.Subscriber('/STAR_pose_continuous', PoseStamped, self.remap_pose_continuous)
        rospy.Subscriber('/STAR_pose', PoseStamped, self.remap_pose)
        #rospy.Subscriber('/transformed_map', OccupancyGrid, self.remap_map)
        self.pose_continuous_pub = rospy.Publisher('/map_pose_continuous', PoseStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/map_pose', PoseStamped, queue_size=10)
        #self.map_pub = rospy.Publisher('/room_map', OccupancyGrid, queue_size=10)

    def remap_pose(self, m):
        m.header.frame_id = "map"
        self.pose_pub.publish(m)

    def remap_pose_continuous(self, m):
        m.header.frame_id = "map"
        self.pose_continuous_pub.publish(m)

    # def remap_map(self, m):
    #     print m.header

    #     m.header.frame_id = "map"
    #     self.map_pub.publish(m)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = TopicRemapper()
    node.run()
