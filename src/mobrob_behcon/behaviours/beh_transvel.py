#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel, DesTransDir

class BehConstTransVel(Behaviour):

    def __init__(self, name, trans_vel=0.2):
        super(BehConstTransVel, self).__init__(name)
        self.trans_vel = trans_vel

    
    def fire(self):
        self.add_desire(DesTransVel(self.trans_vel, 0.5))
        self.add_desire(DesTransDir(0.0, 0.5))
        
