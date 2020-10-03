#!/usr/bin/env python
import rospy
import math
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehTurn(Behaviour):

    def __init__(self, name, degrees=90, rot_vel=0.2):
        super(BehTurn, self).__init__(name)
        self.degrees = self.normalize_angle(degrees)
        self.rot_vel = abs(rot_vel)
        self.init = True
    
    
    def normalize_angle(self, angle):
        newAngle = angle
        while newAngle <= -180: 
            newAngle += 360
        while newAngle > 180: 
            newAngle -= 360
        return newAngle

    
    def fire(self):
        if self.init:
            self.start_pose, _ = self.percept_space.egopose.get_current_Pose()
            self.init = False
        
        current_pose, _ = self.percept_space.egopose.get_current_Pose()

        diff_angle = self.normalize_angle(current_pose[2] - self.start_pose[2])

        rospy.loginfo("BehTurn - currrent=%s, start=%s, diff=%s", str(math.degrees(current_pose[2])), str(math.degrees(self.start_pose[2])), str(math.degrees(diff_angle)))

        if abs(self.degrees - math.degrees(diff_angle)) > 5:
            if self.degrees > 0:
                self.add_desire(DesRotVel(self.rot_vel, 0.6))
            else:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 0.6))
        else:
            rospy.loginfo("BehTurn - success")
            self.add_desire(DesRotVel(0.0, 1.0))
            self.success()
