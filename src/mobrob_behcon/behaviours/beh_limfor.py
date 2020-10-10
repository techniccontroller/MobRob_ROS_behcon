#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel

class BehLimFor(Behaviour):
    """
    The class BehLimFor

    This class is a child of the class Behaviour representing 
    a behaviour which slows down robot as it approaching some obstacle in front.

    """

    def __init__(self, name, stopdistance=0.3, slowdistance=0.5, slowspeed=0.1):
        """
        constructor

        :param name: name of behaviour
        :type name: string
        :param stopdistance: distance at which the robot should stop [m], defaults to 0.3
        :type stopdistance: float, optional
        :param slowdistance: distance at which the robot should slow down [m], defaults to 0.5
        :type slowdistance: float, optional
        :param slowspeed: speed for slow sector, defaults to 0.1
        :type slowspeed: float, optional
        """
        super(BehLimFor, self).__init__(name)
        self.stopdistance = stopdistance
        self.slowdistance = slowdistance
        self.slowspeed = slowspeed

    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # check area in front of robot for obstacles
        dist_front = self.percept_space.laserscanner.check_box(0.250, 0.200, 1.00, -0.200)

        # log important information
        rospy.loginfo("BehLimFor - dist: %s", str(dist_front))

        # create desire for translation speed depending of the distance to obstacle 
        if dist_front > 0:
            if dist_front < self.stopdistance:
                # stop robot
                self.add_desire(DesTransVel(0.0, 1.0))
            elif dist_front < self.slowdistance:
                # slow down robot
                self.add_desire(DesTransVel(self.slowspeed, 1.0))
        
