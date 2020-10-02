#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion

class RelativeMotionNode():
    def __init__(self):
        rospy.init_node('relative_motion')
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.current_pose = None
        self.last_transform_update_time = None

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                transform_time = self.tfBuffer.get_latest_common_time('odom', 'base_link')
                if self.last_transform_update_time is None or transform_time - self.last_transform_update_time > rospy.Duration(1.0):
                    prev_pose_transform_time = self.last_transform_update_time
                    self.last_transform_update_time = transform_time
                    current_transform = self.tfBuffer.lookup_transform('odom', 'base_link', self.last_transform_update_time)
                    rpy = euler_from_quaternion((current_transform.transform.rotation.x,
                                                 current_transform.transform.rotation.y,
                                                 current_transform.transform.rotation.z,
                                                 current_transform.transform.rotation.w))
                    self.prev_pose = self.current_pose
                    # x, y, yaw
                    self.current_pose = (current_transform.transform.translation.x,
                                         current_transform.transform.translation.y,
                                         rpy[2])
                    if self.prev_pose is not None:
                        # this way shows the math, but you can do it more easily with tf2_ros (we will show that too)
                        prev_pose_to_odom = np.array([[np.cos(self.prev_pose[2]), -np.sin(self.prev_pose[2]), self.prev_pose[0]],
                                                      [np.sin(self.prev_pose[2]), np.cos(self.prev_pose[2]), self.prev_pose[1]],
                                                      [0, 0, 1]])
                        current_pose_to_odom = np.array([[np.cos(self.current_pose[2]), -np.sin(self.current_pose[2]), self.current_pose[0]],
                                                         [np.sin(self.current_pose[2]), np.cos(self.current_pose[2]), self.current_pose[1]],
                                                         [0, 0, 1]])
                        # Note: dot is the same as matrix multplication when using a 2 dimensional ndarray
                        current_pose_to_prev_pose = np.linalg.inv(prev_pose_to_odom).dot(current_pose_to_odom)
                        full_transform = self.tfBuffer.lookup_transform_full('base_link', prev_pose_transform_time, 'base_link', self.last_transform_update_time, 'odom')
                        # https://math.stackexchange.com/questions/301319/derive-a-rotation-from-a-2d-rotation-matrix
                        relative_pose_mathy_way = (current_pose_to_prev_pose[0,2],
                                                   current_pose_to_prev_pose[1,2],
                                                   np.arctan2(current_pose_to_prev_pose[1,0],
                                                              current_pose_to_prev_pose[0,0]))
                        rpy_full = euler_from_quaternion((full_transform.transform.rotation.x,
                                                          full_transform.transform.rotation.y,
                                                          full_transform.transform.rotation.z,
                                                          full_transform.transform.rotation.w))
                        relative_pose_transform_full = (full_transform.transform.translation.x,
                                                        full_transform.transform.translation.y,
                                                        rpy_full[2])
                        print(relative_pose_transform_full)
            except Exception as ex:
                print(ex)
                pass
            r.sleep()

if __name__ == '__main__':
    node = RelativeMotionNode()
    node.run()