#!/usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist


class MoveActuator(object):
    """
    The class MoveActuator

    Represents a connector to the move actuator service of the robot.
    Provides multiple functions to control the movement of the robot.
    """

    def __init__(self):
        """
        constructor
        """

        # ROS publisher
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.last_cmd_msg = None
        self.last_send_time_vel = 0

    def send_twist(self, cmd_msg, send_limit=0):
        current_time = MoveActuator.get_current_time()

        diff_to_last = MoveActuator.calc_diff_2d(self.last_cmd_msg, cmd_msg)

        if diff_to_last > 0.01 or current_time - self.last_send_time_vel > send_limit:
            self.pub_cmd_vel.publish(cmd_msg)
            self.last_send_time_vel = current_time
            self.last_cmd_msg = cmd_msg

    @staticmethod
    def calc_diff_2d(twist1, twist2):
        if twist1 is None or twist2 is None:
            return 99
        else:
            return abs(twist1.linear.x - twist2.linear.x) \
                + abs(twist1.linear.y - twist2.linear.y) \
                + abs(twist1.angular.z - twist2.angular.z)

    @staticmethod
    def get_current_time():
        """
        Get current time in milliseconds

        :return: time in milliseconds
        """
        return int(round(time.time() * 1000))
