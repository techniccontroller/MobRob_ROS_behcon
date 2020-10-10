#!/usr/bin/env python
import rospy
import math
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehTurn(Behaviour):
    """
    The class BehTurn

    This class is a child of the class Behaviour representing 
    a behaviour which turns the robot a given angle.

    """

    def __init__(self, name, degrees=90, rot_vel=0.2):
        """
        constructor

        :param name: name of behaviour
        :type name: string
        :param degrees: angle by which the robot should rotate [deg], defaults to 90
        :type degrees: int, optional
        :param rot_vel: rotation speed [m/s], defaults to 0.2
        :type rot_vel: float, optional
        """
        super(BehTurn, self).__init__(name)
        self.degrees = degrees
        self.rot_vel = abs(rot_vel)
        self.init = True
    
    @staticmethod
    def normalize_angle(self, angle):
        """
        Function to normalize an given angle

        :param angle: angle to be normalized [rad]
        :type angle: float
        :return: normalized angle in range [-pi, pi]
        :rtype: float
        """
        newAngle = angle
        while newAngle <= -math.pi: 
            newAngle += 2*math.pi
        while newAngle > math.pi: 
            newAngle -= 2*math.pi
        return newAngle
    
    def fire(self):
        """
        The fire()-method, which is necessary in every Behaviour, 
        will be called by resolver in every polling cycle.
        """
        # during first run save starting position of robot
        if self.init:
            self.start_pose, _ = self.percept_space.egopose.get_current_pose()
            rospy.loginfo("BehTurn - degrees: %s, start_pose: %s,  target:%s", str(self.degrees), str(self.start_pose[2]), str(self.start_pose[2] + math.radians(self.degrees)))
            self.target_orientation = BehTurn.normalize_angle(self.start_pose[2] + math.radians(self.degrees))
            self.init = False
        
        # get current position of robot
        current_pose, _ = self.percept_space.egopose.get_current_pose()

        # calc difference between current and target orientation
        diff_angle = self.target_orientation - current_pose[2]

        # log important infoss
        rospy.loginfo("BehTurn - current=%s, target=%s, diff=%s", str(current_pose[2]), str(self.target_orientation), str(diff_angle))

        # check if difference is larger then tolerance, 
        if abs(math.degrees(diff_angle)) > 5:
            # if so create desire to rotate robot in the necessary direction
            if self.degrees > 0:
                self.add_desire(DesRotVel(self.rot_vel, 0.6))
            else:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 0.6))
        else:
            # if difference is small enough mark this behaviour as successful
            rospy.loginfo("BehTurn - success")
            self.add_desire(DesRotVel(0.0, 1.0))
            self.success()
