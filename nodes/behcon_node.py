#!/usr/bin/env python

import sys
from os import path
#sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )

import rospy
from mobrob_behcon.core.resolver import Resolver
from mobrob_behcon.desires.desires import DesCmdVel
from mobrob_behcon.core.perceptual_space import PerceptualSpace

class BehConNode:

    def __init__(self, id):
        """Initialize class attributes."""
        self.id = id
        rospy.init_node(self.id, anonymous=True)
        self.resolver = Resolver()
        self.lst_behGroups = []
        self.strategy = None
        self.percept_space = PerceptualSpace()
        self.percept_space.add_camera('mobrob_camera', 5001)
        self.percept_space.add_laserscanner('/scan')
        self.percept_space.add_egopose('/odom')

    def start(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            
            self.strategy.plan()
            self.resolver.runOnce()
            
            rate.sleep()

    def add_strategy(self, strategy):
        strategy.set_node(self)
        self.strategy = strategy




if __name__ == '__main__':
    
    print("start")

    rospy.init_node('behcon_node')

    print("node created")

    resolver = Resolver()
    des1 = DesCmdVel([23,45], 0.4)
    des2 = DesCmdVel([80,40], 0.5)
    des3 = DesCmdVel([40,20], 0.5)
    des4 = DesCmdVel([23,60], 0.9)
    des1.set_priority(20)
    des2.set_priority(50)
    des3.set_priority(50)
    des4.set_priority(30)

    lst = []
    lst.append(des1)
    lst.append(des2)
    lst.append(des3)
    lst.append(des4)

    out = resolver.resolveDesire(lst)

    print(out)


    rospy.spin()