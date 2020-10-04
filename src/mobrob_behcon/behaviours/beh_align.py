#!/usr/bin/env python
import rospy
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.desires.desires import DesRotVel

class BehAlign(Behaviour):

    def __init__(self, name, tolerance=0.1, rot_vel=20):
        super(BehAlign, self).__init__(name)
        self.tolerance = tolerance
        self.rot_vel = rot_vel
        self.counter = 0

    
    def fire(self):
        left_dist = self.percept_space.laserscanner.check_box(0.250, 0.100, 0.800, 0.200)
        right_dist = self.percept_space.laserscanner.check_box(0.250, -0.100, 0.800, -0.200)

        print("BehAlign: Hi!" + str(self.counter) + " diff: " + str(left_dist-right_dist) )
        rospy.loginfo("BehAlign - diff: %s", str(left_dist-right_dist))
        self.counter = self.counter + 1

        #if self.counter > 100:
        #    print("BehAlign: Success!")
        #    self.add_desire(DesCmdVel([0,0,0,0,0,0], 1.0))
        #    self.success()

        if left_dist != 0 and right_dist != 0:
            if (left_dist-right_dist) > self.tolerance:
                self.add_desire(DesRotVel(-1 * self.rot_vel, 1.0))
            elif (right_dist-left_dist) > self.tolerance:
                self.add_desire(DesRotVel(self.rot_vel, 1.0))
            else:
                self.add_desire(DesRotVel(0.0, 0.5))
        else:
            self.add_desire(DesRotVel( 0.0, 0.5))
        
