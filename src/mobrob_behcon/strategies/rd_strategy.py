#!/usr/bin/env python

from mobrob_behcon.strategies.strategy import Strategy

class RDStrategy(Strategy):

    def __init__(self, dock):
        super(RDStrategy, self).__init__()
        self.dock = dock
        self.dock.activate_exclusive()

    def plan(self):
        if self.dock.success == 0:
            self.finish = True
