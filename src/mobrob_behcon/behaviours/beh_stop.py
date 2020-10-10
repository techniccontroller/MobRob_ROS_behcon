#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesTransVel

class BehStop(Behaviour):

    def __init__(self, name, stopdistance=0.30):
        super(BehStop, self).__init__(name)
        self.stopdistance = stopdistance

    
    def fire(self):
        dist_front = self.percept_space.laserscanner.check_box(0.250, 0.200, 1.00, -0.200)

        rospy.loginfo("BehStop - dist: %s", str(dist_front))

        if dist_front > 0:
            if dist_front < self.stopdistance:
                # success
                rospy.loginfo("BehStop - Stop success on distance of %s!", str(dist_front))
                self.add_desire(DesTransVel(0.0, 1.0))
                self.success()
        
