#!/usr/bin/env python

import rospy
from mobrob_behcon.core.resolver import Resolver
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.core.visualisation import KOOSVisu
from mobrob_behcon.visu.visu_behcon import VisuBehCon


class BehConNode:
    """
    The class BehConNode

    This class represents the central container of all components.
    An object of this class serves as a link between the components 
    of the behavior pattern control: 

    - Strategy

    - Resolver

    - PreceptualSpace

    - List of BehaviourGroups

    - VisuBehCon (Visualisation for Strategy)

    - KOOSVisu (Visualistation for PerceptualSpace)
   
    """

    def __init__(self, id):
        """ 
        constructor

        :param id: name of object
        :type id: string
        """
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
        self.visubehcon = VisuBehCon(self)
        print("node created")

    def start(self):
        """ 
        Start the execution of the robot tasks
        """
        self.visubehcon.draw()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            
            self.strategy.plan()           

            self.resolver.runOnce()
            ego_pose, _ = self.percept_space.egopose.get_current_pose()
            self.percept_space.visu.set_current_pose(ego_pose)
            self.percept_space.visu.draw_robot()
            self.percept_space.visu.draw_points_laser(self.percept_space.laserscanner.get_lst_scan_points()[0])
            self.visu.send_image()
            self.visubehcon.update()

            if self.strategy.is_finished():
                rospy.loginfo("Strategy finished -> shutdown")
                rospy.signal_shutdown("Finished execution")
            rate.sleep()

    def add_strategy(self, strategy):
        """
        add Strategy to the robot control software

        :param strategy: strategy object to be added
        :type strategy: Strategy
        :return: returns nothing
        """
        strategy.set_node(self)
        self.strategy = strategy
    
    def add_beh_group(self, beh_group):
        """
        add BehaviourGroup to the robot control software

        :param strategy: BehaviourGroup object to be added
        :type strategy: BehaviourGroup
        :return: returns nothing
        """
        beh_group.set_resolver(self.resolver)
        beh_group.set_percept_space(self.percept_space)
        self.lst_behGroups.append(beh_group)