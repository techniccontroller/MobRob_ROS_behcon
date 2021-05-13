#!/usr/bin/env python
import rospy
from mobrob_behcon.strategies.strategy import Strategy

class RDStrategy(Strategy):
    """
    The class RDStrategy

    The class is a child of the abstract class Strategy 
    and manages the strategic level of the behaviours based control 
    of the application **R**\ ightAngle\ **D**\ ock as statemachine.
    
    """

    def __init__(self, dock, turn90):
        """
        constructor

        :param dock: a BehaviourGroup containing the necessary behaviours for first task
        :type dock: BehaviourGroup
        :param turn90: a BehaviourGroup containing the necessary behaviours for second task
        :type turn90: BehaviourGroup
        """
        super(RDStrategy, self).__init__()
        self.dock = dock
        self.turn90 = turn90
        self.add_behgrp(dock)
        self.add_behgrp(turn90)
        self.activate_exclusive(dock)

    def plan(self):
        """
        plan() method, where the magic happens :). It contains and manages the state machine. 
        """
        rospy.loginfo("Strategy - dock=%s, turn90=%s", str(self.dock.success), str(self.turn90.success))
        if self.dock.success == 0:
            self.dock.success = -1
            self.activate_exclusive(self.turn90)
        if self.turn90.success == 0:
            self.finish = True
        if self.dock.success == 99:
            # emergency stop
            rospy.loginfo("success==99 -> emergency stop")
            self.finish = True
