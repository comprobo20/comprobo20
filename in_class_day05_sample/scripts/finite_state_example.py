#!/usr/bin/env python3

""" This code implements the finite state controller described in https://comprobo20.github.io/in-class/day05.

    There are three states: moving forward, moving backward, and turning left.  The initial state is moving_forward.
    The rules for transitioning between these states are as follows.

    moving_forward:
        - if a bump sensor is activated transition to moving backward
    moving_backward:
        - once the obstacle in front of the robot is greater than a threshold, transition to turning left
    turning_left:
        - once 1 second has elapsed, transition to moving_forward
"""


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan

class FiniteStateController():
    """ This class encompasses the finite state controller ROS Node"""
    def __init__(self):
        rospy.init_node('finite_state_example')
        # the angular velocity when in the "turning_left" state
        self.angular_velocity = 0.5
        # the forward speed when in the "moving_forward" state.  The speed is reversed in the "moving_backward" state.
        self.forward_speed = 0.1
        # the distance threshold to use to transition from "moving_backward" to turning left
        self.distance_threshold = 0.8
        # a class attribute to track the distance to the obstacle in front of the robot.  The last valid measurement
        # is always stored (where valid means not equal to 0).
        self.distance_to_obstacle = 0.0
        # keeps track of whether the robot's bump sensors are active
        self.bump_active = False
        # holds the current state (note: the type of self.state is actually a function!)
        self.state = self.moving_forward

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('bump', Int8MultiArray, self.process_bump)
        rospy.Subscriber('scan', LaserScan, self.process_scan)

    def process_bump(self, msg):
        """ Handle bump messages by noting whether any of the sensors are active """
        self.bump_active = msg.data[0] == 1 or msg.data[1] == 1 or msg.data[2] == 1 or msg.data[3] == 1

    def process_scan(self, msg):
        """ Handle laser can messages by storing the distance to the obstacle immediately in front of the robot """
        if msg.ranges[0] != 0:
            self.distance_to_obstacle = msg.ranges[0]

    def run(self):
        """ The run loop repeatedly executes the current state function.  Each state function will return a function
            corresponding to the next state to run. """
        # this sleep is to allow any subscribers to cmd_vel to establish a connection to our publisher.  This is only
        # needed in the case where you send the velocity commands once (in some ways sending the commands repeatedly is
        # more robust.
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state = self.state()

    def moving_forward(self):
        """ Implements the moving forward state.  The next state to run is returned by the function. """
        m = Twist()
        m.linear.x = self.forward_speed
        self.vel_pub.publish(m)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.bump_active:
                return self.moving_backward
            r.sleep()

    def moving_backward(self):
        """ Implements the moving backward state.  The next state to run is returned by the function. """
        m = Twist()
        m.linear.x = -self.forward_speed
        self.vel_pub.publish(m)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # check distance to forward obstacle to decide when we've reversed "enough"
            if self.distance_to_obstacle > self.distance_threshold:
                return self.rotating_left
            r.sleep()

    def rotating_left(self):
        """ Implements the rotating left state.  The next state to run is returned by the function. """
        m = Twist()
        m.angular.z = self.angular_velocity
        self.vel_pub.publish(m)
        # the transition to the next state is purely time-based, so we can use rospy.sleep
        rospy.sleep(1.0)
        return self.moving_forward

if __name__ == '__main__':
    node = FiniteStateController()
    node.run()
