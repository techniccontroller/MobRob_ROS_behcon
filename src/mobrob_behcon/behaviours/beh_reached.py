#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel

class BehReached(Behaviour):
    """
    The class BehReached

    This class is a child of the class Behaviour representing 
    a behaviour which stops the robot when it reached some obstacle in front.

    """

    def __init__(self, name, stopdistance=0.30):
        """
        constructor

        :param name: name of behaviour
        :type name: string
        :param stopdistance: distance on with the robot should stop, defaults to 0.30
        :type stopdistance: float, optional
        """
        super(BehReached, self).__init__(name)
        self.stopdistance = stopdistance

    
    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # check front sides of robot for obstacles
        dist_front = self.percept_space.laserscanner.check_box(0.250, 0.500, 0.530, -0.500)

        do_stop = False

        # check if on one side is a obstacle nearer than stopdistance
        if dist_front > 0:
            if dist_front < self.stopdistance:
                do_stop = True
                rospy.loginfo("BehStop - Stop because of obstacle in front in distance of %s!", str(dist_front))
        
        if do_stop:
            # robot need to stop because of obstacle, 
            # create desire and set behaviour as success code 0
            self.add_desire(DesTransVel(0.0, 1.0))
            self.success(0)
        
