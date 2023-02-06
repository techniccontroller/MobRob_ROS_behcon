#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from mobrob_behcon.utils.mobrob_transformation import *
from mobrob_slam.utils.graphical_functions import cov2elli


class KOOSVisu(object):
    """
    The class KOOSVisu

    This class represents a visualization of the robot environment 
    from a bird's eye view. The different classes like Laser Scanner or 
    Behaviour can draw on the canvas. 
    Output is published as a video stream via a ROS Publisher.
    """

    colours = [(0, 0, 255),
               (0, 255, 0),
               (255, 0, 255),
               (0, 255, 255),
               (255, 0, 0),
               (255, 255, 0)]

    def __init__(self):
        """
        constructor
        """
        self.image_pub = rospy.Publisher("image_koosvisu", Image, queue_size=10)
        self.bridge = CvBridge()
        self.width = 600
        self.height = 600
        self.scale = 200.0
        self.clear_image()
        self.current_pose = (0.0, 0.0, 0.0)

    def clear_image(self):
        """
        Clears the canvas.
        """
        # create empty image
        self.img = np.zeros([self.width, self.height, 3], dtype=np.uint8)

    def send_image(self):
        """
        Sends out the canvas via the ROS publisher.
        """
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
        self.clear_image()

    def to_visu_coord(self, point):
        """
        Converts a point in world coordinates to pixel coordinates on canvas.

        :param point: point to be converted to pixel coord
        :type point: number[2]
        :return: pixel coordinates on canvas
        :rtype: (int, int)
        """
        point = np.array(point).reshape(2, -1)
        x = point[0, :]
        y = point[1, :]
        x_center = self.width / 2
        y_center = self.height / 2
        x_visu = x_center - (y * self.scale)
        y_visu = y_center - (x * self.scale)

        if point.size == 2:
            return int(x_visu[0]), int(y_visu[0])
        else:
            return np.vstack((x_visu, y_visu)).astype(np.int32)

    def set_current_pose(self, current_pose):
        """
        Setter for robots current pose

        :param current_pose: current pose of robot in world space (x, y, yaw)
        :type current_pose: (float, float, float)
        """
        self.current_pose = current_pose

    def draw_origin_world(self):
        line_width = 2  # int(0.001 * self.scale)
        # X axis
        cv2.arrowedLine(self.img,
                        self.to_visu_coord((0, 0)),
                        self.to_visu_coord((1, 0)),
                        color=(0, 0, 255),
                        thickness=line_width)
        # Y axis
        cv2.arrowedLine(self.img,
                        self.to_visu_coord((0, 0)),
                        self.to_visu_coord((0, 1)),
                        color=(0, 255, 0),
                        thickness=line_width)

    def draw_robot(self, color=(0, 255, 0), current_pose=None):
        """
        Draws the robot on the canvas as rectangle with the given color.

        :param color: color of robot on canvas, defaults to (0, 255, 0)
        :type color: (byte, byte, byte)
        :param current_pose: current pose of robot in world space (x, y, yaw), defaults to None
        :type current_pose: (float, float, float)
        """
        if current_pose is not None:
            self.set_current_pose(current_pose)
        LF, RF, RB, LB = get_robot_corners(self.current_pose)
        LM = (np.array(LF) + np.array(LB)) / 2
        RM = (np.array(RF) + np.array(RB)) / 2
        FM = (np.array(LF) + np.array(RF)) / 2
        line_width = 2  # int(0.001 * self.scale)
        # rect
        cv2.line(self.img, self.to_visu_coord(LF), self.to_visu_coord(RF), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(RF), self.to_visu_coord(RB), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(RB), self.to_visu_coord(LB), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(LB), self.to_visu_coord(LF), color, thickness=line_width)
        # triangle
        cv2.line(self.img, self.to_visu_coord(FM), self.to_visu_coord(RM), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(RM), self.to_visu_coord(LM), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(LM), self.to_visu_coord(FM), color, thickness=line_width)

    def draw_box(self, pt1, pt2, color=(255, 0, 0)):
        """
        Draws a box on the canvas. Box is defined by two points in robot space.

        :param pt1: Right back point of box in robot space (x, y)
        :type pt1: (float, float)
        :param pt2: Left front point of box in robot space (x, y)
        :type pt2: (float, float)
        :param color: color of box on canvas, defaults to (255, 0, 0)
        :type color: (byte, byte, byte)
        """
        LF = get_world_coordinate(self.current_pose, pt2[0], pt2[1])
        RF = get_world_coordinate(self.current_pose, pt2[0], pt1[1])
        RB = get_world_coordinate(self.current_pose, pt1[0], pt1[1])
        LB = get_world_coordinate(self.current_pose, pt1[0], pt2[1])
        line_width = 2  # int(0.001 * self.scale)
        cv2.line(self.img, self.to_visu_coord(LF), self.to_visu_coord(RF), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(RF), self.to_visu_coord(RB), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(RB), self.to_visu_coord(LB), color, thickness=line_width)
        cv2.line(self.img, self.to_visu_coord(LB), self.to_visu_coord(LF), color, thickness=line_width)

    def draw_point_laser(self, point, color=(255, 255, 255)):
        """
        Draws a dot on canvas. The dot is defined by one point in laser space.

        :param point: point in laser space (x, y)
        :type point: (float, float)
        :param color: color of dot on canvas, defaults to (255, 255, 255)
        :type color: (byte, byte, byte)
        """
        point = np.array(point).flatten()
        assert point.size == 2, "wrong shape of laser point" + str(point)
        if abs(point[0]) < 100 and abs(point[1]) < 100:
            (x_robot, y_robot) = get_robot_coordinate_from_laser(point[0], point[1])
            # print((x_robot, y_robot))
            (x_visu, y_visu) = self.to_visu_coord(get_world_coordinate(self.current_pose, x_robot, y_robot))
            # (x_visu, y_visu) = self.to_visu_coord(get_world_coordinate(self.current_pose, point[0], point[1]))
            radius = int(0.005 * self.scale)
            if 0 < x_visu < self.width and 0 < y_visu < self.height:
                cv2.circle(self.img, (x_visu, y_visu), radius, color, -1)

    def draw_points_laser(self, lst_points, color=(255, 255, 255)):
        """
        Draws a list of points to canvas.

        :param lst_points: list of points (defined in laser space)
        :type lst_points: list of (float, float)
        :param color: color of dot on canvas, defaults to (255, 255, 255)
        :type color: (byte, byte, byte)
        """
        if isinstance(lst_points, list):
            for i in range(len(lst_points)):
                self.draw_point_laser(lst_points[i], color)
        else:
            lst_points = np.array(lst_points)
            assert lst_points.shape[0] == 2, "point array as wrong shape, should be (2, -1)"
            for i in range(lst_points.shape[1]):
                self.draw_point_laser(lst_points[:, i], color)

    def draw_cluster_points_laser(self, points, cluster_labels):
        """
        Draws a list of clustered points to canvas. 
        Six different colors are available to color the clusters differently.

        :param points: list of points (defined in laser space)
        :type points: list of (float, float)
        :param cluster_labels: list of cluster labels, the list has to have the same length as points
        :type cluster_labels: list of int
        """
        for i in range(len(points)):
            if cluster_labels[i] == -1:
                self.draw_point_laser(points[i], (255, 255, 255))
            else:
                self.draw_point_laser(points[i], self.colours[cluster_labels[i] % 6])

    def draw_point_world(self, point, color=(255, 255, 255)):
        """
        Draws a dot on canvas. The dot is defined by one point in world space.

        :param point: point in world space (x, y)
        :type point: (float, float)
        :param color: color of dot on canvas, defaults to (255, 255, 255)
        :type color: (byte, byte, byte)
        """
        point = np.array(point).flatten()
        assert point.size == 2, "wrong shape of point" + str(point)
        if abs(point[0]) < 100 and abs(point[1]) < 100:
            (x_visu, y_visu) = self.to_visu_coord(point)
            radius = int(0.005 * self.scale)
            if 0 < x_visu < self.width and 0 < y_visu < self.height:
                cv2.circle(self.img, (x_visu, y_visu), radius, color, -1)

    def draw_points_world(self, lst_points, color=(255, 255, 255)):
        """
        Draws a list of points to canvas. points are defined in world space

        :param lst_points: list of points (defined in world space)
        :type lst_points: list of (float, float)
        :param color: color of dot on canvas, defaults to (255, 255, 255)
        :type color: (byte, byte, byte)
        """
        for i in range(len(lst_points)):
            self.draw_point_world(lst_points[i], color)

    def draw_line_world(self, slope, offset, color=(0, 0, 255)):
        x_max = self.width / self.scale
        x = np.arange(-x_max, x_max)
        y = slope * x + offset
        line_width = 2  # int(0.001 * self.scale)
        cv2.line(self.img,
                 self.to_visu_coord((x[0], y[0])),
                 self.to_visu_coord((x[-1], y[-1])),
                 color,
                 thickness=line_width)

    def draw_cross_world(self, point, color=(128, 128, 0), size=2):
        point = np.array(point).flatten()
        assert point.size == 2, "wrong shape of point" + str(point)
        if abs(point[0]) < 100 and abs(point[1]) < 100:
            (x_visu, y_visu) = self.to_visu_coord(point)
            if 0 < x_visu < self.width and 0 < y_visu < self.height:
                line_width = size  # int(0.001 * self.scale)
                cv2.line(self.img,
                         (x_visu + 10 * size, y_visu),
                         (x_visu - 10 * size, y_visu),
                         color,
                         thickness=line_width)
                cv2.line(self.img,
                         (x_visu, y_visu + 10 * size),
                         (x_visu, y_visu - 10 * size),
                         color,
                         thickness=line_width)

    def draw_crosses_world(self, lst_points, color=(255, 255, 255), size=2):
        """
        Draws a list of crosses to canvas.

        :param lst_points: list of points (defined in world space)
        :type lst_points: list of (float, float)
        :param color: color of dot on canvas, defaults to (255, 255, 255)
        :type color: (byte, byte, byte)
        :param size: size of crosses, defaults to 2
        :type: int
        """
        if isinstance(lst_points, list):
            for i in range(len(lst_points)):
                self.draw_cross_world(lst_points[i], color, size)
        else:
            lst_points = np.array(lst_points)
            assert lst_points.shape[0] == 2, "point array as wrong shape, should be (2, -1), is " + str(lst_points.shape)
            for i in range(lst_points.shape[1]):
                self.draw_cross_world(lst_points[:, i], color, size)

    def draw_ellipse_world(self, point, covariance, sigma, color=(0, 128, 128)):
        point = np.array(point).flatten()
        assert point.size == 2, "wrong shape of point" + str(point)
        assert covariance.shape == (2, 2), "wrong shape of covariance" + str(covariance)
        xx, yy = cov2elli(point, covariance, sigma, 16)
        pts = self.to_visu_coord(np.vstack((xx, yy))).T.reshape((-1, 1, 2))
        line_width = 2  # int(0.001 * self.scale)
        cv2.polylines(self.img, [pts], isClosed=True, color=color, thickness=line_width)

    def draw_text_world(self, point, text, color=(255, 255, 255)):
        point = np.array(point).flatten()
        assert point.size == 2, "wrong shape of point" + str(point)
        if abs(point[0]) < 100 and abs(point[1]) < 100:
            (x_visu, y_visu) = self.to_visu_coord(point)
            if 0 < x_visu < self.width and 0 < y_visu < self.height:
                cv2.putText(self.img, text, (x_visu, y_visu), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

