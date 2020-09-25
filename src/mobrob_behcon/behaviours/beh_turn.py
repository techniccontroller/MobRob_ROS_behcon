#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehTurn(Behaviour):

    def __init__(self, name, degrees=90, rot_vel=0.2):
        super(BehTurn, self).__init__(name)
        self.degrees = degrees
        self.rot_vel = abs(rot_vel)
        self.init = True

    
    def fire(self):
        if self.init:
            self.start_pose, _ = self.percept_space.egopose.get_current_Pose()
            self.init = False
        
        current_pose, _ = self.percept_space.egopose.get_current_Pose()

        diff_angle = current_pose[2] - self.start_pose[2]

        if abs(self.degrees - diff_angle) > 5:
            if self.degrees > 0:
                self.add_desire(DesRotVel(self.rot_vel, 1.0))
            else:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 1.0))
