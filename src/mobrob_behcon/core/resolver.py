#!/usr/bin/env python

#import sys
#from os import path
#sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )

import rospy
from geometry_msgs.msg import Twist
from mobrob_behcon.desires.desire import Desire
from mobrob_behcon.desires.desires import DesCmdVel, DesCmdLight, DesCmdGripper
from mobrob_behcon.behaviours.behaviour import Behaviour
import copy

class Resolver:

	""" 
	The class Resolver 

	It resolves a output for the actuators from given desires. 
	Currently it understands desires fo following type:

	- DesCmdVel

	- DesCmdLight

	- DesCmdGripper
	"""

	def __init__(self):
		"""
		Construct a new 'Resolver' object. 

		:return: returns nothing
		"""
		self.lst_behaviours = []
		self.lst_desires_vel =  []
		self.lst_desires_grip =  []
		self.lst_desires_led = []

		# ROS publisher
		self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)


	def add_desire(self, desire):
		"""
		Add new desire to resolver

		:param desire: a desire to be added to the resolver
		:type desire: Desire
		:return: returns nothing
		"""
		if isinstance(desire, DesCmdVel):
			self.lst_desires_vel.append(desire)
		elif isinstance(desire, DesCmdGripper):
			self.lst_desires_grip.append(desire)
		elif isinstance(desire, DesCmdLight):
			self.lst_desires_led.append(desire)
		else:
			print("Type of desire not known: " + type(desire).__name__)


	def reset(self):
		"""
		Reset all lists (desires and behaviours)

		"""
		self.lst_behaviours = []
		self.lst_desires_vel = []
		self.lst_desires_grip = []
		self.lst_desires_led = []


	def set_behaviour_lst(self, behaviours):
		"""
		set the new active behaviour list

		:param lst_behaviour: list of behaviour which are currently active
		:type lst_behaviour: list(Behaviour)
		"""
		self.lst_behaviours = behaviours


	def resolveDesire(self, lst_desires):
		"""
		function to resolve a value from a list of desires 
		
		:param lst_desires: a list of desires (all items of the same type)
		:type lst_desires: list[Desire]
		:return: returns the resolved value (depending on type of value in desire, can also be an array)
		"""

		output_value = 0

		if len(lst_desires) > 0:

			# sort list of desires based on priority (high to low)
			lst_desires.sort(key=lambda x: x.priority, reverse=True)

			# print type of desires and whole list
			print("Desire-Type: " + type(lst_desires[0]).__name__)
			for d in lst_desires:
				print(d)
			
			# create resulting desire (mostly used to track the value and strength during next calculations)
			result_desire = Desire(0, 0)
			# declare variable to count desires with the same priority
			num_desire_prio = 0
			# declare index variable for outer while loop
			i = 0

			while result_desire.strength < 1.0 and i < len(lst_desires):
				# reset counter variable
				num_desire_prio = 0
				# create temporary desire to calculate weighted mean of desires with same priority
				temp_desire = Desire(0, 0)
				for k in range(len(lst_desires)):
					# loop through list of desires and find desires with same priority
					if lst_desires[k].priority == lst_desires[i].priority:
						# add weighted value and strength of selected desire to temporary desire
						temp_desire.value = temp_desire.value + lst_desires[k].value * lst_desires[k].strength
						temp_desire.strength = temp_desire.strength + lst_desires[k].strength
						num_desire_prio = num_desire_prio + 1
				# calc average of the strengths (maybe not the right way?!)
				#temp_desire.strength = temp_desire.strength / num_desire_prio
				# add strengths and value to result_desire
				result_desire.value = result_desire.value + temp_desire.value
				result_desire.strength = result_desire.strength + temp_desire.strength
				# jump over all desires with the same priority in list
				i = i + num_desire_prio

			output_value = result_desire.value / result_desire.strength
			print("Output: ", output_value)

		return output_value

	def runOnce(self):
		"""
		Run one cycle of the resolver to calculate output values 
		and publish corresponding messages

		"""

		

		# resolve desires to get velocity values
		cmd_value = self.resolveDesire(self.lst_desires_vel)
		cmd_msg = Twist()
		cmd_msg.linear.x = cmd_value[0]
		cmd_msg.linear.y = cmd_value[1]
		cmd_msg.linear.z = cmd_value[2]
		cmd_msg.angular.x = cmd_value[3]
		cmd_msg.angular.y = cmd_value[4]
		cmd_msg.angular.z = cmd_value[5]

		# publish velocity command
		self.pub_cmd_vel.publish(cmd_msg)
		
