#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time

class LaserScanner(object):
    """
    The class LaserScanner

    Represents a connector to the laserscanner topics of the robot.
    Provides a function to get the latest laserscan from the laserscanner on the robot.
    """

    def __init__(self, ros_topic):
        """
        constructor

        :param ros_topic: topic of the LaserScan data
        :type ros_topic: string
        """
        self.ros_topic = ros_topic
        self.sub = rospy.Subscriber(self.ros_topic, LaserScan, self.callback)
        self.dist_list = None
        self.last_scan_time = 0

    def callback(self, msg):
        """
        callback function for topics of type sensor_msgs/LaserScan

        :param msg: receiving message
        """
        self.list = LaserScanner.extract_points(msg.ranges)
        self.last_scan_time = LaserScanner.get_current_time()

    def get_dist_list(self):
        """
        Get current laserscan as a list of points

        :return: laserscan as a list of points (x,y)
        :rtype: list((float, float))
        """
        age = LaserScanner.get_current_time() - self.last_scan_time
        return self.dist_list, age

    @staticmethod
    def get_x(radius, angle):
        """
        Get x-coordinate of given point in polar coordinates

        :param radius: radius [m]
        :type radius: float
        :param angle: angle [deg]
        :type angle: float
        :return: x-coordinate [m]
        :rtype: float
        """
        return np.cos(np.deg2rad(angle))*radius
    
    @staticmethod
    def get_y(radius, angle):
        """
        Get y-coordinate of given point in polar coordinates

        :param radius: radius [m]
        :type radius: float
        :param angle: angle [deg]
        :type angle: float
        :return: y-coordinate [m]
        :rtype: float
        """
        return np.sin(np.deg2rad(angle))*radius

    @staticmethod
    def extract_points(ranges):
        """Extract points from a list of distances

        :param ranges: list of distances (length=360)
        :type ranges: list(float)
        :return: list of points (x, y)
        :rtype: list((float, float))
        """
        dist_list = []
        for i in range(len(ranges)):
            value = np.nan_to_num(ranges[i])
            center_coordinates = (LaserScanner.get_x(value, i), LaserScanner.get_y(value, i))
            dist_list.append(center_coordinates)            
        
        return dist_list
    
    @staticmethod
    def filter_points(list, x_min, x_max, y_min, y_max):
        """
        Filter given list of points (x, y) with the given bounderies 

        :param list: list of points (x, y)
        :type list: list((float, float))
        :param x_min: lower bound x
        :type x_min: float
        :param x_max: upper bound x
        :type x_max: float
        :param y_min: lower bound y
        :type y_min: float
        :param y_max: upper bound y
        :type y_max: float
        :return: filtered list of points (x, y)
        :rtype: list((float, float))
        """
        filt_list = []
        for i in range(len(list)):
            if (list[i][0] > x_min) and (list[i][0] < x_max)\
                and (list[i][1] > y_min) and (list[i][1] < y_max):
                filt_list.append(list[i])
        
        return filt_list

    @staticmethod
    def get_current_time():
        """
        Get current time in milliseconds

        :return: time in milliseconds
        """     
        return int(round(time.time() * 1000))
