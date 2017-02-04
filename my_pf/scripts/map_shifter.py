#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Vector3
import tf
from copy import deepcopy

class MapShifter(object):
	def __init__(self):
		rospy.init_node('map_shifter')
		rospy.Subscriber('/map', OccupancyGrid, self.process_map)
		self.target_frame = rospy.get_param('~target_frame', 'STAR')
		self.tf_listener = tf.TransformListener()
		self.map_pub = rospy.Publisher('/transformed_map', OccupancyGrid, queue_size=10)

	def process_map(self, m):
		print m.header.stamp
		self.tf_listener.waitForTransform(self.target_frame, m.header.frame_id, m.header.stamp, rospy.Duration(1.0))
		trans, rot = self.tf_listener.lookupTransform(self.target_frame, m.header.frame_id, m.header.stamp)
		yaw = tf.transformations.euler_from_quaternion(rot)[2]
		# TODO: handle orientation of the map
		new_pose = self.tf_listener.transformPose(self.target_frame,
									  			  PoseStamped(header=m.header,
									  	          			  pose=Pose(position=Vector3(x=m.info.origin.position.x,
									  	          							 			 y=m.info.origin.position.y))))
		new_msg = deepcopy(m)
		# shift the map so it is consistent with the positions we have so far.
		#new_msg.header.frame_id = self.target_frame
		new_msg.info.origin.position = new_pose.pose.position
		new_msg.info.origin.orientation = new_pose.pose.orientation
		self.map_pub.publish(new_msg)

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			r.sleep()

if __name__ == '__main__':
	node = MapShifter()
	node.run()