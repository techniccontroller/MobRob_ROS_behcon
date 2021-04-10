#!/usr/bin/env python

class BehaviourGroup(object):
    """
    The class BehaviourGroup

    """

    def __init__(self, name):
        """
        constructor

        :param name: name of behaviourgroup
        :type name: string
        """
        #: Name of behaviourgroup
        self.name = name  
        #: resolver  
        self.resolver = None  
        #: percept_space
        self.percept_space = None  
        self.lst_behaviours = []
        self.success = -1
        self.error = -1

    def set_resolver(self, resolver):
        """
        Set the reference to resolver

        :param resolver: relevant resolver object
        :type resolver: Resolver
        """
        self.resolver = resolver
        for beh_cap in self.lst_behaviours:
            beh_cap.behaviour.set_resolver(resolver)
    
    def set_percept_space(self, percept_space):
        """
        Set the reference to percept_space

        :param percept_space: relevant percept_space object
        :type percept_space: PerceptualSpace
        """
        self.percept_space = percept_space
        for beh_cap in self.lst_behaviours:
            beh_cap.behaviour.set_percept_space(percept_space)

    def add(self, behaviour, priority):
        """
        function to add an behaviour to behaviourgroup

        :param behaviour: reference to behaviour
        :type behaviour: inherited class of Behaviour
        :param priority: priority of behaviour in this behaviourgroup [0-100]
        :type priority: int
        """
        behaviour.set_resolver(self.resolver)
        behaviour.set_percept_space(self.percept_space)
        self.lst_behaviours.append(BehaviourGroup.BehaviourCapsula(behaviour, priority))

    def activate_exclusive(self):
        """
        Activate this behaviourgroup, 
        so set all behaviours in this behaviourgroup as active so that resolver can trigger them 
        """
        self.success = -1
        self.error = -1
        lst_active_behaviours = []
        for beh_cap in self.lst_behaviours:
            beh_cap.behaviour.set_priority(beh_cap.priority)
            beh_cap.behaviour.set_current_beh_group(self)
            lst_active_behaviours.append(beh_cap.behaviour)
        self.resolver.set_behaviour_lst(lst_active_behaviours)

    def set_success(self, code):
        """
        Set success state of this behaviourgroup.
        With the success code some more information 
        about the success can be given.

        :param code: success code, defaults to 0
        :type code: int, optional
        """
        if code > self.success:
            self.success = code
    
    def set_error(self, code):
        """
        Set error state of this behaviourgroup.
        With the error code some more information 
        about the error can be given.

        :param code: error code, defaults to 0
        :type code: int, optional
        """
        if code > self.error:
            self.error = code
    

    class BehaviourCapsula(object):
        """
        The class BehaviourCapsula

        this class wraps a behaviour together with a priority 
        inside behaviourgroup
        """

        def __init__(self, behaviour, priority):
            """
            constructor

            :param behaviour: reference to behaviour
            :type behaviour: inherited class of Behaviour
            :param priority: priority of behaviour [0-100]
            :type priority: int
            """
            self.behaviour = behaviour
            self.priority = priority