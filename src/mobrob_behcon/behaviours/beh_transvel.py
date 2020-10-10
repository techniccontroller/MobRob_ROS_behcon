#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel, DesTransDir

class BehConstTransVel(Behaviour):
    """
    The class BehConstTransVel

    This class is a child of the class Behaviour representing 
    a behaviour which drives the robot with constant speed.

    """

    def __init__(self, name, trans_vel=0.2, trans_dir=0.0):
        """
        constructor

        :param name: name of behaviour
        :type name: string
        :param trans_vel: speed at which the robot should move [m/s], defaults to 0.2
        :type trans_vel: float, optional
        :param trans_dir: direction in which the robot should move [deg], defaults to 0.0
        :type trans_dir: float, optional
        """
        super(BehConstTransVel, self).__init__(name)
        self.trans_vel = trans_vel
        self.trans_dir = trans_dir
 
    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # create desires to move the robot in given direction with given speed
        self.add_desire(DesTransVel(self.trans_vel, 0.5))
        self.add_desire(DesTransDir(self.trans_dir, 0.5))
        
