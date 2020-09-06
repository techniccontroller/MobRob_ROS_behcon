#!/usr/bin/env python

import rospy
from mobrob_behcon.behaviours.behaviourgroup import BehaviourGroup
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.behaviours.beh_align import BehAlign
from mobrob_behcon.strategies.rd_strategy import RDStrategy
from mobrob_behcon.core.behcon_node import BehConNode


if __name__ == '__main__':
    
    print("start")
    raw_input("Press Enter to continue...")

    robot = BehConNode("mobrob_node")

    dock = BehaviourGroup("Dock")
    dock2 = BehaviourGroup("Dock2")
    turn90 = BehaviourGroup("turn90")
    #BehConstTransVel cv = new BehConstTransVel("ConstTransVel", 200);
    #BehTurn tu = new BehTurn("Turn", -90);
    #BehLimFor lf = new BehLimFor("LimFor", 400, 2000, 100); 
    al = BehAlign("Align", tolerance=0.030, rot_vel=0.5)
    #BehStop st = new BehStop("Stop", 800);
    #BehSaveRADock srfull = new BehSaveRADock("SaveDock1", 1000, 1000);
    #BehSaveRADock srright = new BehSaveRADock("SaveDock2", 300, 1000);

    #dock.add(lf, 80);
    #dock.add(cv, 50);
    dock.add(al, 75)
    #dock.add(st, 80);
    #dock.add(srfull, 90);
    robot.add_beh_group(dock)
    
    #dock2.add(lf, 80);
    #dock2.add(cv, 50);
    dock2.add(al, 75)
    #dock2.add(st, 80);
    #dock2.add(srright, 90);
    robot.add_beh_group(dock2)
    
    #turn90.add(tu, 80);
    turn90.add(al, 80)
    robot.add_beh_group(turn90)
    
    robot.add_strategy(RDStrategy(dock))
    
    
    robot.start()		  