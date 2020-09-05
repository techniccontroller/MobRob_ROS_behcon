#!/usr/bin/env python

import sys
from os import path
#sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )

import rospy
from mobrob_behcon.core.resolver import Resolver
from mobrob_behcon.desires.desires import DesCmdVel
from mobrob_behcon.core.perceptual_space import PerceptualSpace
from mobrob_behcon.behaviours.behaviourgroup import BehaviourGroup
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.behaviours.beh_align import BehAlign
from mobrob_behcon.strategies.rd_strategy import RDStrategy

class BehConNode:

    def __init__(self, id):
        """Initialize class attributes."""
        self.id = id
        rospy.init_node(self.id, anonymous=True)
        self.resolver = Resolver()
        self.lst_behGroups = []
        self.strategy = None
        self.percept_space = PerceptualSpace()
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
            rate.sleep()

    def add_strategy(self, strategy):
        strategy.set_node(self)
        self.strategy = strategy
    
    def add_beh_group(self, beh_group):
        beh_group.set_resolver(self.resolver)
        beh_group.set_percept_space(self.percept_space)
        self.lst_behGroups.append(beh_group)





if __name__ == '__main__':
    
    print("start")
    raw_input("Press Enter to continue...")

    #rospy.init_node('behcon_node')

    #print("node created")

    robot = BehConNode("mobrob_node")

    dock = BehaviourGroup("Dock");
    dock2 = BehaviourGroup("Dock2");
    turn90 = BehaviourGroup("turn90");
    #BehConstTransVel cv = new BehConstTransVel("ConstTransVel", 200);
    #BehTurn tu = new BehTurn("Turn", -90);
    #BehLimFor lf = new BehLimFor("LimFor", 400, 2000, 100); 
    al = BehAlign("Align", 30, 5);
    #BehStop st = new BehStop("Stop", 800);
    #BehSaveRADock srfull = new BehSaveRADock("SaveDock1", 1000, 1000);
    #BehSaveRADock srright = new BehSaveRADock("SaveDock2", 300, 1000);

    #dock.add(lf, 80);
    #dock.add(cv, 50);
    dock.add(al, 75);
    #dock.add(st, 80);
    #dock.add(srfull, 90);
    robot.add_beh_group(dock);
    
    #dock2.add(lf, 80);
    #dock2.add(cv, 50);
    dock2.add(al, 75);
    #dock2.add(st, 80);
    #dock2.add(srright, 90);
    robot.add_beh_group(dock2);
    
    #turn90.add(tu, 80);
    turn90.add(al, 80);
    robot.add_beh_group(turn90);
    
    robot.add_strategy(RDStrategy(dock));
    
    
    robot.start()		  