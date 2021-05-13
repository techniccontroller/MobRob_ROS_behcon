#!/usr/bin/env python

from mobrob_behcon.connectors.laserscanner import LaserScanner
from mobrob_behcon.connectors.camera import CameraTCP
from mobrob_behcon.connectors.egopose import EgoPose

class PerceptualSpace(object):
    """
    The class PerceptualSpace

    This contains all sensors of the robot. Other parts of the program
    can refer to this to get sensor data.

    Currently following sensors are supported:

    - Laserscanner

    - Egopose sensor

    - Camera
    """

    def __init__(self, visu):
        """constructor

        :param visu: an object of KOOSVisu
        :type visu: KOOSVisu
        """
        # define different sensor connectors
        self.laserscanner = None
        self.camera = None 
        self.egopose = None 
        self.visu = visu

    def add_laserscanner(self, ros_topic="/scan"):
        """
        Add laserscanner to the configuration

        :param ros_topic: topic of the LaserScan data
        :type ros_topic: string
        :return: returns nothing
        """
        self.laserscanner = LaserScanner(ros_topic, self.visu)
    
    def add_camera(self, ip_address="mobrob", port=5001):
        """
        Add camera (TCP) to the configuration. A camera server needs to be running on the network for that. 

        e.g. following: https://github.com/techniccontroller/MobRob_PI_Scripts/blob/master/Camera/videoTCPServer8_first_capture.py 

        :param ip_address: ip address of camera server
        :param port: port of camera server
        :type ip_address: string
        :type port: int
        :return: returns nothing
        """
        self.camera = CameraTCP(ip_address, port)

    def add_egopose(self, ros_topic="/odom"):
        """
        Add egopose sensor to the configuration. 
        This will provide information about the current pose of the robot.

        :param ros_topic: possible topics are **'/pose'** (myrobot_model/Pose) or **'/odom'** (nav_msgs/Odometry)
        :type ros_topic: string
        :return: returns nothing
        """
        self.egopose = EgoPose(ros_topic)
    

