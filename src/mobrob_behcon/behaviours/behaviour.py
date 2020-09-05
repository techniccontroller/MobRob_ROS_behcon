#!/usr/bin/env python

from abc import ABCMeta, abstractmethod

class Behaviour():
    """
    The abstract class Behaviour

    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        """
        Constructor

        :param name: name or behaviour
        :type name: string
        """
        #: Name of behaviour
        self.name = name  
        #: resolver  
        self.resolver = None  
        #: percept_space
        self.percept_space = None 
        #: priority of behaviour  
        self.priority = 0 
        #: current active behaviour group  
        self.current_beh_group = None   

    def set_resolver(self, resolver):
        """
        Set the reference to resolver

        :param resolver: relevant resolver object
        :type resolver: Resolver
        """
        self.resolver = resolver
    
    def set_percept_space(self, percept_space):
        """
        Set the reference to percept_space

        :param percept_space: relevant percept_space object
        :type percept_space: PerceptualSpace
        """
        self.percept_space = percept_space
    
    def set_priority(self, priority):
        """
        Set current priority of the behaviour, 
        can be changed during runtime

        :param priority: [description]
        :type priority: [type]
        """
        self.priority = priority
    
    def set_current_beh_group(self, beh_group):
        """
        Set current active behaviourgroup

        :param beh_group: the currently active behaviour group
        :type beh_group: BehaviourGroup
        """
        self.current_beh_group = beh_group

    @abstractmethod
    def fire(self):
        """
        Abstract method which will 
        be called in the cyclic polling loop.

        Needs to be defined by inherited classes. 

        In this method you can call the function add_desire()
        to send a desire to the resolver. 
        Depending on strength and priority this will be 
        forwarded to actuators
        """
        pass

    def add_desire(self, desire):
        """
        Send desire to resolver.

        :param desire: a desire to be added to the resolver
        :type desire: Desire
        """
        desire.set_priority(self.priority)
        self.resolver.add_desire(desire)
    
    def success(self, code=0):
        """
        Set the behaviour as successful ended.
        With the success code some more information 
        about he success can be given to the behaviourgroup

        :param code: success code, defaults to 0
        :type code: int, optional
        """
        self.current_beh_group.set_success(code)

    def error(self, code=0):
        """
        Raise an error about the behaviour.
        With the error code some more information 
        about the error can be given to the behaviourgroup

        :param code: error code, defaults to 0
        :type code: int, optional
        """
        self.current_beh_group.set_error(code)


        


