#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel

class BehStop(Behaviour):
    """
    The class BehStop

    This class is a child of the class Behaviour representing 
    a behaviour which stops the robot when some obstacle from any side is approaching.

    """

    def __init__(self, name, stopdistance=0.30):
        """
        constructor

        :param name: name of behaviour
        :type name: string
        :param stopdistance: distance on with the robot should stop, defaults to 0.30
        :type stopdistance: float, optional
        """
        super(BehStop, self).__init__(name)
        self.stopdistance = stopdistance

    
    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # check all sides of robot for obstacles
        dist_front = self.percept_space.laserscanner.check_box(0.250, 0.500, 0.530, -0.500)
        dist_back = self.percept_space.laserscanner.check_box(-0.250, 0.500, -0.530, -0.500)
        dist_left = self.percept_space.laserscanner.check_box(-0.250, 0.500, 0.250, 0.120)
        dist_right = self.percept_space.laserscanner.check_box(-0.250, -0.120, 0.250, -0.500)

        do_stop = False

        # check if on one side is a obstacle nearer than stopdistance
        if dist_front > 0:
            if dist_front < self.stopdistance:
                do_stop = True
                rospy.loginfo("BehStop - Stop because of obstacle in front in distance of %s!", str(dist_front))
        if dist_back > 0:
            if dist_back < self.stopdistance:
                do_stop = True
                rospy.loginfo("BehStop - Stop because of obstacle in back in distance of %s!", str(dist_back))
        if dist_left > 0:
            if dist_left < self.stopdistance:
                do_stop = True
                rospy.loginfo("BehStop - Stop because of obstacle on left in distance of %s!", str(dist_left))
        if dist_right > 0:
            if dist_right < self.stopdistance:
                do_stop = True
                rospy.loginfo("BehStop - Stop because of obstacle on right in distance of %s!", str(dist_right))
        
        if do_stop:
            # robot need to stop because of obstacle, 
            # create desire and set behaviour as success with code 99 -> emergency stop
            self.add_desire(DesTransVel(0.0, 1.0))
            self.success(99)
        
