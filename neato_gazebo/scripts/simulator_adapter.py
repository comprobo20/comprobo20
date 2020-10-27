#!/usr/bin/env python3

# TODO: it would be nice if raw_vel actually changed single wheel velocities

from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Int8MultiArray
import rospy

class RawVelRelayNode(object):
    def __init__(self):
        # it is less important that this wheel base is accurate for the simulated robot but that it is true to the real robot
        self.real_robot_wheel_base = 0.235
        # Note: from neato_description/neato_gazebo.urdf.xacro <wheelDiameter>0.07</wheelDiameter> (this might not be physically accurate, but it is what is there for the differential drive controller
        self.wheel_radius = 0.07/2
        rospy.init_node('simulator_adapter')
        rospy.Subscriber('imu', Imu, self.imu_received)
        rospy.Subscriber('raw_vel', Float32MultiArray, self.raw_vel_received)
        rospy.Subscriber('joint_states', JointState, self.joint_states_received)
        rospy.Subscriber('bumper', ContactsState, self.contacts_received)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.accel_pub = rospy.Publisher('accel', Float32MultiArray, queue_size=10)
        self.encoders_pub = rospy.Publisher('encoders', Float32MultiArray, queue_size=10)
        self.bumper_pub = rospy.Publisher('bump', Int8MultiArray, queue_size=10)

    def raw_vel_received(self, msg):
        print(msg.data)
        if len(msg.data) == 2:
            vL, vR = msg.data[0], msg.data[1]
            msg = Twist()
            msg.linear.x = (vL + vR)/2;
            msg.angular.z = (vR - vL)/self.real_robot_wheel_base;
            self.cmd_vel_pub.publish(msg)

    def imu_received(self, msg):
        self.accel_pub.publish(
            Float32MultiArray(data=[msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z]))


    def joint_states_received(self, msg):
        self.encoders_pub.publish(
            Float32MultiArray(data=[msg.position[0]*self.wheel_radius,
                                    msg.position[1]*self.wheel_radius]))

    def contacts_received(self, msg):
        # NOTE: this is oversimplified (just uses front bumper in a binary fashion)
        if len(msg.states):
            self.bumper_pub.publish(
                Int8MultiArray(data=[1,1,1,1]))
        else:
            self.bumper_pub.publish(
                Int8MultiArray(data=[0,0,0,0]))

if __name__ == '__main__':
    node = RawVelRelayNode()
    rospy.spin()
