#!/usr/bin/env python

import rospy
import time
from myrobot_model.msg import Pose
from nav_msgs.msg import Odometry
import tf

class EgoPose(object):
    """
    The class EgoPose

    Represents a connector to the positioning topics of the robot.
    Provides a function to get the current pose of the robot.
    """

    def __init__(self, ros_topic):
        """
        constructor

        :param ros_topic: possible topics are **'/pose'** (myrobot_model/Pose) or **'/odom'** (nav_msgs/Odometry)
        :type ros_topic: string
        """
        self.ros_topic = ros_topic
        if self.ros_topic == '/odom':
            self.sub = rospy.Subscriber(self.ros_topic, Odometry, self.callbackOdometry)
        elif self.ros_topic == '/pose':
            self.sub = rospy.Subscriber(self.ros_topic, Pose, self.callbackPose)
        self.current_pose = None
        self.last_pose_time = 0

    def callbackPose(self, msg):
        """
        callback function for topics of type myrobot_model/Pose

        :param msg: receiving message
        """
        self.current_pose = (msg.x, msg.y, msg.theta)
        self.last_pose_time = EgoPose.get_current_time()
    
    def callbackOdometry(self, msg):
        """
        callback function for topics of type nav_msgs/Odometry

        :param msg: receiving message
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        explicit_quat = [msg.pose.pose.orientation.x, \
                            msg.pose.pose.orientation.y, \
                            msg.pose.pose.orientation.z, \
                            msg.pose.pose.orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        #_, _, yaw = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (x, y, yaw)
        self.last_pose_time = EgoPose.get_current_time()

    def get_current_Pose(self):
        """
        Get current pose of robot

        :return: current pose (x, y, yaw), age of data (milliseconds)
        :rtype: (float, float, float), int
        """
        age = EgoPose.get_current_time() - self.last_pose_time
        return self.current_pose, age
    
    @staticmethod
    def get_current_time():
        """
        Get current time in milliseconds

        :return: time in milliseconds
        """  
        return int(round(time.time() * 1000))