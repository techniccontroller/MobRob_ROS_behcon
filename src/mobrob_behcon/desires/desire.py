#!/usr/bin/env python

import numpy as np

class Desire(object):
	"""
	The class Desire

	A desire consist of 

	- a value (any number/array),

	- a dynamic strength (0.0 - 1.0) and 

	- a static priority (0 - 100)

	The priority will be set when Desire is forwarded to resolver.
	"""

	def __init__(self, value, strength):
		"""
		Construct a new 'Desire' object. 
		
		:param value: the numberic value of the desire, can be also be a array/list
		:param strength: the strength of the desire 

		:return: returns nothing
		"""

		self.strength = strength
		self.priority = 0
		self.value = np.array(value)

	def set_priority(self, priority):
		"""
		Setter for attribute priority 

		:param priority: priority to be set

		:return: returns nothing
		"""

		self.priority = priority

	def __str__(self):
		"""
		Create string representation of a Desire object

		:return: string representation of a Desire object
		"""
		return "[v: {}, s: {}, p: {}]".format(self.value, self.strength, self.priority)


