#!/usr/bin/env python

import rospy
from mobrob_behcon.behaviours.behaviourgroup import BehaviourGroup
from mobrob_behcon.behaviours.behaviour import Behaviour
from mobrob_behcon.behaviours.beh_align import BehAlign
from mobrob_behcon.behaviours.beh_transvel import BehConstTransVel
from mobrob_behcon.behaviours.beh_limfor import BehLimFor
from mobrob_behcon.behaviours.beh_stop import BehStop
from mobrob_behcon.behaviours.beh_turn import BehTurn
from mobrob_behcon.behaviours.beh_reached import BehReached
from mobrob_behcon.strategies.rd_strategy import RDStrategy
from mobrob_behcon.core.behcon_node import BehConNode


if __name__ == '__main__':
    
    print("start")
    input("Press Enter to continue...")

    robot = BehConNode("mobrob_node")

    dock = BehaviourGroup("Dock")
    dock2 = BehaviourGroup("Dock2")
    turn90 = BehaviourGroup("turn90")
    cv = BehConstTransVel("ConstTransVel", trans_vel=0.2)
    tu = BehTurn("Turn", -90)
    lf = BehLimFor("LimFor", stopdistance=0.2, slowdistance=0.5, slowspeed=0.1)
    al = BehAlign("Align", tolerance=0.020, rot_vel=0.2)
    st = BehStop("Stop", stopdistance=0.3)
    re = BehReached("Reached", stopdistance=0.4)
    #BehSaveRADock srfull = new BehSaveRADock("SaveDock1", 1000, 1000);
    #BehSaveRADock srright = new BehSaveRADock("SaveDock2", 300, 1000);

    dock.add(lf, 80)
    dock.add(cv, 50)
    dock.add(al, 75)
    dock.add(st, 80)
    dock.add(re, 60)
    #dock.add(srfull, 90)
    robot.add_beh_group(dock)
    
    #dock2.add(lf, 80);
    #dock2.add(cv, 50);
    #dock2.add(al, 75)
    #dock2.add(st, 80);
    #dock2.add(srright, 90);
    #robot.add_beh_group(dock2)
    
    turn90.add(tu, 80)
    #turn90.add(al, 80)
    robot.add_beh_group(turn90)

    robot.add_beh_group(dock)

    robot.add_beh_group(turn90)
    
    robot.add_strategy(RDStrategy(dock, turn90))
    
    
    robot.start()		  