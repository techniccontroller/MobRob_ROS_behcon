#!/usr/bin/env python
import rospy
from mobrob_behcon.strategies.strategy import Strategy

class RDStrategy(Strategy):

    def __init__(self, dock, turn90):
        super(RDStrategy, self).__init__()
        self.dock = dock
        self.turn90 = turn90
        self.add_behgrp(dock)
        self.add_behgrp(turn90)
        self.activate_exclusive(dock)

    def plan(self):
        rospy.loginfo("Strategy - dock=%s, turn90=%s", str(self.dock.success), str(self.turn90.success))
        if self.dock.success == 0:
            self.dock.success = -1
            self.activate_exclusive(turn90)
        if self.turn90.success == 0:
            self.finish = True
