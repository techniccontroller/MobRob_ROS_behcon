#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel

class BehLimFor(Behaviour):

    def __init__(self, name, stopdistance=0.2, slowdistance=0.5, slowspeed=0.1):
        super(BehLimFor, self).__init__(name)
        self.stopdistance = stopdistance
        self.slowdistance = slowdistance
        self.slowspeed = slowspeed

    
    def fire(self):
        dist_front = self.percept_space.laserscanner.check_box(0.100, 0.200, 1.00, -0.200)

        print("BehLimFor - dist: " + str(dist_front) )
        rospy.loginfo("BehLimFor - dist: %s", str(dist_front))

        if dist_front > 0:
            if dist_front < self.stopdistance:
                # stop robot
                self.add_desire(DesTransVel(0.0, 1.0))
            elif dist_front < self.slowdistance:
                # slow down robot
                self.add_desire(DesTransVel(self.slowspeed, 1.0))
        
