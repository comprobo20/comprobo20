#!/usr/bin/env python3

""" This code implements the finite state controller described in https://comprobo20.github.io/in-class/day05.
    In this version we will use the ROS smach library
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
import smach
# if you want to try to the smach viewer, consider uncommenting these lines
# import smach_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan

class FiniteStateControllerSmach():
    """ This class encompasses the finite state controller ROS Node"""
    def __init__(self):
        rospy.init_node('finite_state_example')
        # the angular velocity when in the "turning_left" state
        self.angular_velocity = 0.5
        # the forward speed when in the "moving_forward" state.  The speed is reversed in the "moving_backward" state.
        self.forward_speed = 0.1
        # the distance threshold to use to transition from "moving_backward" to turning left
        self.distance_threshold = 0.8

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def run(self):
        # needed in the case where you send the velocity commands once (in some ways sending the commands repeatedly is
        # more robust.
        rospy.sleep(1)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=[])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('moving_forward', MovingForward(self.forward_speed, self.vel_pub),
                                   transitions={'bumped object': 'moving_backward'})
            smach.StateMachine.add('moving_backward', MovingBackward(self.vel_pub, self.forward_speed, self.distance_threshold),
                                   transitions={'obstacle far enough away': 'rotating_left'})
            smach.StateMachine.add('rotating_left', RotatingLeft(self.vel_pub, self.angular_velocity),
                                   transitions={'rotated for 1 second': 'moving_forward'})

        # Create and start the introspection server
        # if you want to try smach viewer, consider uncommenting these lines
        # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        # sis.start()
        # Execute SMACH plan
        outcome = sm.execute()

class MovingForward(smach.State):
    """ Implements the moving forward state. """

    def __init__(self, forward_speed, vel_pub):
        self.forward_speed = forward_speed
        self.vel_pub = vel_pub
        # keeps track of whether the robot's bump sensors are active
        self.bump_active = False
        rospy.Subscriber('bump', Int8MultiArray, self.process_bump)
        smach.State.__init__(self, outcomes=['bumped object'])

    def process_bump(self, msg):
        """ Handle bump messages by noting whether any of the sensors are active """
        self.bump_active = msg.data[0] == 1 or msg.data[1] == 1 or msg.data[2] == 1 or msg.data[3] == 1

    def execute(self, user_data):
        m = Twist()
        m.linear.x = self.forward_speed
        self.vel_pub.publish(m)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.bump_active:
                return 'bumped object'
            r.sleep()

class MovingBackward(smach.State):
    """ Implements the moving backward state. """

    def __init__(self, vel_pub, forward_speed, distance_threshold):
        self.vel_pub = vel_pub
        self.forward_speed = forward_speed
        self.distance_threshold = distance_threshold
        # a class attribute to track the distance to the obstacle in front of the robot.  The last valid measurement
        # is always stored (where valid means not equal to 0).
        self.distance_to_obstacle = 0.0
        rospy.Subscriber('scan', LaserScan, self.process_scan)
        smach.State.__init__(self, outcomes=['obstacle far enough away'])

    def process_scan(self, msg):
        """ Handle laser can messages by storing the distance to the obstacle immediately in front of the robot """
        if msg.ranges[0] != 0:
            self.distance_to_obstacle = msg.ranges[0]

    def execute(self, user_data):
        m = Twist()
        m.linear.x = -self.forward_speed
        self.vel_pub.publish(m)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # check distance to forward obstacle to decide when we've reversed "enough"
            if self.distance_to_obstacle > self.distance_threshold:
                return 'obstacle far enough away'
            r.sleep()

class RotatingLeft(smach.State):
    """ Implements the rotating left state.  The next state to run is returned by the function. """

    def __init__(self,  vel_pub, angular_velocity):
        self.vel_pub = vel_pub
        self.angular_velocity = angular_velocity
        smach.State.__init__(self, outcomes=['rotated for 1 second'])

    def execute(self, user_data):
        m = Twist()
        m.angular.z = self.angular_velocity
        self.vel_pub.publish(m)
        # the transition to the next state is purely time-based, so we can use rospy.sleep
        rospy.sleep(1.0)
        return 'rotated for 1 second'

if __name__ == '__main__':
    node = FiniteStateControllerSmach()
    node.run()
