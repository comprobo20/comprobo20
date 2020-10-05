#!/usr/bin/env python3

""" This code uses the tf2_ros module to compute the relative motion of the
    Neato.  In this code we show two different (but equivalent approaches).
    (1) We use two different base_link -> odom transforms (sampled at different         points in time) and combine them to figure out the motion of the robot.
    (2) We use the lookup_transform_full command to get the motion all at once.

    As a last step, we show that the two approaches do indeed yield the same
    results.
"""

import rospy
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion

class RelativeMotionNode():
    def __init__(self):
        rospy.init_node('relative_motion')
        # tf_buffer and tf_listener are both needed to perform transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # current_pose stores the pose of the robot that was most recently
        # computed.  The format of the pose is an (x,y,theta) tuple.
        self.current_pose = None
        # this is the timestamp at which current_pose corresponds.
        self.last_transform_update_time = None

    @staticmethod
    def xy_theta_to_homogeneous_transform(p):
        # https://math.stackexchange.com/questions/301319/derive-a-rotation-from-a-2d-rotation-matrix
        return np.array([[np.cos(p[2]), -np.sin(p[2]), p[0]],
                         [np.sin(p[2]), np.cos(p[2]), p[1]],
                         [0, 0, 1]])

    @staticmethod
    def homogeneous_transform_to_xy_theta(T):
        return T[0,2], T[1,2], np.arctan2(T[1,0], T[0,0])

    @staticmethod
    def transform_to_xy_theta(T):
        return (T.translation.x,
                T.translation.y,
                RelativeMotionNode.yaw_from_transform(T))

    @staticmethod
    def yaw_from_transform(T):
        return euler_from_quaternion((T.rotation.x,
                                      T.rotation.y,
                                      T.rotation.z,
                                      T.rotation.w))[2]

    @staticmethod
    def transform_to_xy_theta(T):
        yaw = RelativeMotionNode.yaw_from_transform(T)
        return (T.translation.x, T.translation.y, yaw)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                transform_time = self.tf_buffer.get_latest_common_time('odom', 'base_link')
                if self.last_transform_update_time is None or transform_time - self.last_transform_update_time > rospy.Duration(1.0):
                    prev_pose_transform_time = self.last_transform_update_time
                    prev_pose = self.current_pose
                    self.last_transform_update_time = transform_time
                    current_transform = \
                        self.tf_buffer.lookup_transform('odom',
                                                        'base_link',
                                                        self.last_transform_update_time).transform
                    self.current_pose = RelativeMotionNode.transform_to_xy_theta(current_transform)
                    if prev_pose is not None:
                        # this way shows the math, but you can do it more easily with tf2_ros (we will show that too)
                        prev_pose_to_odom = RelativeMotionNode.xy_theta_to_homogeneous_transform(prev_pose)
                        current_pose_to_odom = RelativeMotionNode.xy_theta_to_homogeneous_transform(self.current_pose)
                        # Note: dot is the same as matrix multiplication when using a 2 dimensional ndarray
                        current_pose_to_prev_pose = np.linalg.inv(prev_pose_to_odom).dot(current_pose_to_odom)
                        full_transform = self.tf_buffer.lookup_transform_full('base_link',
                                                                              prev_pose_transform_time,
                                                                              'base_link',
                                                                              self.last_transform_update_time,
                                                                              'odom').transform
                        motion_way_1 = RelativeMotionNode.homogeneous_transform_to_xy_theta(current_pose_to_prev_pose)
                        print("method 1", motion_way_1)
                        motion_way_2 = RelativeMotionNode.transform_to_xy_theta(full_transform)
                        print("methdo 2", motion_way_2)
            except Exception as ex:
                print(ex)
                pass
            r.sleep()

if __name__ == '__main__':
    node = RelativeMotionNode()
    node.run()
