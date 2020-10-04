#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mobrob_behcon.utils.mobrob_transformation import *

class KOOSVisu(object):

    colours = [(0, 0, 255), 
           (0, 255, 0),
           (255, 0, 255), 
           (0, 255, 255), 
           (255, 0, 0),
           (255, 255, 0)]


    def __init__(self):
        self.image_pub = rospy.Publisher("image_koosvisu",Image,queue_size=10)
        self.bridge = CvBridge()
        self.width = 600
        self.height = 600
        self.scale = 200.0
        self.clear_image()
        self.current_pose = (0.0, 0.0, 0.0)
    
    def clear_image(self):
        # create empty image
        self.img = np.zeros([self.width, self.height, 3], dtype=np.uint8)
    
    def send_image(self):
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
        self.clear_image()
    
    def to_visu_coord(self, point):
        x = point[0]
        y = point[1]
        x_center = self.width/2
        y_center = self.height/2
        x_visu = x_center - (y * self.scale)
        y_visu = y_center - (x * self.scale)
        return (int(x_visu), int(y_visu))

    def set_current_pose(self, current_pose):
        self.current_pose = current_pose   


    def draw_robot(self, color = (0, 255, 0)):
        LF, RF, RB, LB = get_robot_corners(self.current_pose)
        linewidth = 2#int(0.001 * self.scale)
        print(LF)
        cv2.line(self.img, self.to_visu_coord(LF), self.to_visu_coord(RF), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(RF), self.to_visu_coord(RB), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(RB), self.to_visu_coord(LB), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(LB), self.to_visu_coord(LF), color, thickness=linewidth)
    
    def draw_box(self, pt1, pt2, color = (255, 0, 0)):
        LF = get_world_coordinate(self.current_pose, pt2[0], pt2[1])
        RF = get_world_coordinate(self.current_pose, pt2[0], pt1[1])
        RB = get_world_coordinate(self.current_pose, pt1[0], pt1[1])
        LB = get_world_coordinate(self.current_pose, pt1[0], pt2[1])
        linewidth = 2#int(0.001 * self.scale)
        cv2.line(self.img, self.to_visu_coord(LF), self.to_visu_coord(RF), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(RF), self.to_visu_coord(RB), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(RB), self.to_visu_coord(LB), color, thickness=linewidth)
        cv2.line(self.img, self.to_visu_coord(LB), self.to_visu_coord(LF), color, thickness=linewidth)

    def draw_point(self, point, color=(255, 255, 255)):
        if abs(point[0]) < 100 and abs(point[1]) < 100:
            (x_robot, y_robot) = get_robot_coordinate(point[0], point[1])
            #print((x_robot, y_robot))
            (x_visu, y_visu) = self.to_visu_coord(get_world_coordinate(self.current_pose, x_robot, y_robot))
            #(x_visu, y_visu) = self.to_visu_coord(get_world_coordinate(self.current_pose, point[0], point[1]))
            radius = int(0.005 * self.scale)
            if x_visu > 0 and x_visu < self.width and y_visu > 0 and y_visu < self.height:
                cv2.circle(self.img, (x_visu, y_visu), radius, color, -1)

    def draw_points(self, list, color=(255, 255, 255)):
        for i in range(len(list)):
            self.draw_point(list[i], color)

    def draw_cluster_points(self, points, cluster_labels):
        for i in range(len(points)):
            if cluster_labels[i] == -1:
                self.draw_point(points[i], (255, 255, 255))
            else:
                self.draw_point(points[i], self.colours[cluster_labels[i]%6])

    
