#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehAlign(Behaviour):
    """
    The class BehAlign

    This class is a child of the class Behaviour representing 
    a behaviour which align the robot orthogonally to wall.

    """

    def __init__(self, name, tolerance=0.1, rot_vel=20):
        """[summary]

        :param name: name of behaviour
        :type name: string
        :param tolerance: tolerance error angle, defaults to 0.1
        :type tolerance: float, optional
        :param rot_vel: rotation speed, defaults to 20
        :type rot_vel: int, optional
        """
        super(BehAlign, self).__init__(name)
        self.tolerance = tolerance
        self.rot_vel = rot_vel

    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # check area in left front and right front of robot for distance to wall
        left_dist = self.percept_space.laserscanner.check_box(0.250, 0.100, 0.800, 0.200)
        right_dist = self.percept_space.laserscanner.check_box(0.250, -0.100, 0.800, -0.200)

        # log important information
        rospy.loginfo("BehAlign - diff: %s", str(left_dist-right_dist))

        # if both distance are not zero in both area the wall was detected
        if left_dist != 0 and right_dist != 0:
            # according to difference in both distances create desires to rotate
            if (left_dist-right_dist) > self.tolerance:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 1.0))
            elif (right_dist-left_dist) > self.tolerance:
                self.add_desire(DesRotVel(self.rot_vel, 1.0))
            else:
                # if difference is inside tolerance stop rotation
                self.add_desire(DesRotVel(0.0, 0.5))
        else:
            # if no wall is detected stop rotation
            self.add_desire(DesRotVel( 0.0, 0.5))
        
