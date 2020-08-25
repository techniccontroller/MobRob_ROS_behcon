#!/usr/bin/env python

from abc import ABCMeta, abstractmethod

class Behaviour():
    """
    The abstract class Behaviour

    """
    __metaclass__ = ABCMeta

    def __init__(self, id):
        self.id = id
        


