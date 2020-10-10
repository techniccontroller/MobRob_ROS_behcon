#!/usr/bin/env python
import rospy
import math
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehTurn(Behaviour):

    def __init__(self, name, degrees=90, rot_vel=0.2):
        super(BehTurn, self).__init__(name)
        self.degrees = degrees
        self.rot_vel = abs(rot_vel)
        self.init = True
    
    def normalize_angle(self, angle):
        newAngle = angle
        while newAngle <= -math.pi: 
            newAngle += 2*math.pi
        while newAngle > math.pi: 
            newAngle -= 2*math.pi
        return newAngle
    
    def fire(self):
        if self.init:
            self.start_pose, _ = self.percept_space.egopose.get_current_pose()
            rospy.loginfo("BehTurn - degrees: %s, start_pose: %s,  target:%s", str(self.degrees), str(self.start_pose[2]), str(self.start_pose[2] + math.radians(self.degrees)))
            self.target_orientation = self.normalize_angle(self.start_pose[2] + math.radians(self.degrees))
            self.init = False
        
        current_pose, _ = self.percept_space.egopose.get_current_pose()

        diff_angle = self.target_orientation - current_pose[2]

        rospy.loginfo("BehTurn - current=%s, target=%s, diff=%s", str(current_pose[2]), str(self.target_orientation), str(diff_angle))

        if abs(math.degrees(diff_angle)) > 5:
            if self.degrees > 0:
                self.add_desire(DesRotVel(self.rot_vel, 0.6))
            else:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 0.6))
        else:
            rospy.loginfo("BehTurn - success")
            self.add_desire(DesRotVel(0.0, 1.0))
            self.success()
