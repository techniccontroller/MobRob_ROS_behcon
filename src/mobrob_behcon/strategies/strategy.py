#!/usr/bin/env python

from abc import ABCMeta, abstractmethod

class Strategy():
    """
    The abstract class Strategy

    Every inherited class needs a method plan(), 
    which will be called in every polling cycle 
    and will update the built in statemachine.

    Every state of the statemachine consist of a behaviourgroup.
    The method plan() will set, depending on the status 
    of the the different behaviourgroups, those behaviourgroup as active.

    """
    __metaclass__ = ABCMeta
    lst_behgrps = []

    def __init__(self):
        self.beh_node = None
        self.finish = False

    def set_node(self, beh_node):
        """
        Sets the active BehConNode, where it has been added to.

        :param beh_node: node of type BehConNode
        
        :return: returns nothing
        """
        self.beh_node = beh_node
    
    @abstractmethod
    def plan(self):
        """
        This method is called in every polling cycle.
        
        Needs to be implemented by child-class to update statemachine.
        """
        pass

    def is_finished(self):
        """
        Returns if stratgy in finish-state

        :return: True if statemachine is in finish-state, else False
        """
        return self.finish

    def add_behgrp(self, behgrp):
        """
        add behaviour sroup to strategy

        :param behgrp: behaviour group
        :type behgrp: BehaviourGroup
        """
        self.lst_behgrps.append(behgrp)

    def activate_exclusive(self, behgrp):
        """
        activate given behaviour group (need to be added before to strategy)

        :param behgrp: behaviour group
        :type behgrp: BehaviourGroup
        """

        for bg in self.lst_behgrps:
            if bg != behgrp:
                bg.deactivate()
        behgrp.activate_exclusive()