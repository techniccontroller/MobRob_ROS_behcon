#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time
import math
from mobrob_behcon.utils.mobrob_transformation import *

class LaserScanner(object):
    """
    The class LaserScanner

    Represents a connector to the laserscanner topics of the robot.
    Provides a function to get the latest laserscan from the laserscanner on the robot.
    """

    def __init__(self, ros_topic, visu):
        """
        constructor

        :param ros_topic: topic of the LaserScan data
        :type ros_topic: string
        """
        self.ros_topic = ros_topic
        self.sub = rospy.Subscriber(self.ros_topic, LaserScan, self.callback)
        self.lst_scan_points = []
        self.last_scan_time = 0
        self.visu = visu

    def callback(self, msg):
        """
        callback function for topics of type sensor_msgs/LaserScan

        :param msg: receiving message
        """
        self.lst_scan_points = LaserScanner.extract_points(msg.ranges)
        self.last_scan_time = LaserScanner.get_current_time()

    def get_lst_scan_points(self):
        """
        Get current laserscan as a list of points

        :return: laserscan as a list of points (x,y)
        :rtype: list((float, float))
        """
        age = LaserScanner.get_current_time() - self.last_scan_time
        return self.lst_scan_points, age

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
        lst_scan_points = []
        for i in range(len(ranges)):
            value = np.nan_to_num(ranges[i])
            center_coordinates = (LaserScanner.get_x(value, i), LaserScanner.get_y(value, i))
            lst_scan_points.append(center_coordinates)            
        
        return lst_scan_points

    @staticmethod
    def calc_dist(x, y):
        return math.sqrt(x*x + y*y)
    
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
    
    def check_box(self, x1, y1, x2, y2):
        """
        Check if some obstacle in given box

        :param x1: x-coordinate of first corner of box
        :type x1: float
        :param y1: y-coordinate of first corner of box
        :type y1: float
        :param x2: x-coordinate of second corner of box
        :type x2: float
        :param y2: y-coordinate of second corner of box
        :type y2: float
        :return: distance to nearest point in this box, if no obstacle in box returns 0
        :rtype: float
        """
        self.visu.draw_box((x1, y1), (x2, y2))
        x1, y1 = get_laser_coordinate(x1, y1)
        x2, y2 = get_laser_coordinate(x2, y2)
        bound_x_low = x2 if x1 > x2 else x1
        bound_x_high = x1 if x1 > x2 else x2
        bound_y_low = y2 if y1 > y2 else y1
        bound_y_high = y1 if y1 > y2 else y2

        lst_scan_points, _ = self.get_lst_scan_points()
        lst_scan_points_filt = self.filter_points(lst_scan_points, bound_x_low, bound_x_high, bound_y_low, bound_y_high)
        nearest_point = (0, 0)
        if len(lst_scan_points_filt) > 0:
            nearest_point = min(lst_scan_points_filt, key = lambda p: self.calc_dist(p[0], p[1]))

        return self.calc_dist(nearest_point[0], nearest_point[1])
    
    def draw_laserpoints(self):
        rospy.loginfo("Laserscanner - numPoints=%d", len(self.lst_scan_points))
        self.visu.draw_points(self.lst_scan_points)

