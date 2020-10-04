#!/usr/bin/env python

import rospy
from mobrob_behcon.core.resolver import Resolver
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.core.visualisation import KOOSVisu


class BehConNode:

    def __init__(self, id):
        """Initialize class attributes."""
        self.id = id
        rospy.init_node(self.id, anonymous=True)
        self.resolver = Resolver()
        self.lst_behGroups = []
        self.strategy = None
        self.visu = KOOSVisu()
        self.percept_space = PerceptualSpace(self.visu)
        #self.percept_space.add_camera('mobrob_camera', 5001)
        self.percept_space.add_laserscanner('/scan')
        self.percept_space.add_egopose('/odom')
        print("node created")

    def start(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            
            self.strategy.plan()

            if self.strategy.is_finished():
                rospy.signal_shutdown("Finished execution")

            self.resolver.runOnce()
            ego_pose, _ = self.percept_space.egopose.get_current_pose()
            #self.percept_space.visu.set_current_pose(ego_pose)
            self.percept_space.visu.draw_robot()
            self.percept_space.laserscanner.draw_laserpoints()
            self.visu.send_image()
            rate.sleep()

    def add_strategy(self, strategy):
        strategy.set_node(self)
        self.strategy = strategy
    
    def add_beh_group(self, beh_group):
        beh_group.set_resolver(self.resolver)
        beh_group.set_percept_space(self.percept_space)
        self.lst_behGroups.append(beh_group)