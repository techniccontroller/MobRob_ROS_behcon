#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mobrob_behcon.desires.desire import Desire
from mobrob_behcon.desires.desires import DesCmdVel, DesTransVel, DesTransDir, DesRotVel
from mobrob_behcon.connectors.moveactuator import MoveActuator
import math

class Resolver:
	"""
	The class Resolver 

	It resolves a output for the actuators from given desires. 
	Currently it understands desires fo following type:

	- DesCmdVel

	"""

	def __init__(self):
		"""
		Construct a new 'Resolver' object. 

		:return: returns nothing
		"""
		self.lst_behaviours = []
		self.lst_desires_vel = []
		self.lst_des_transvel = []
		self.lst_des_rotvel = []
		self.lst_des_transdir = []

		self.moveactuator = MoveActuator()


	def add_desire(self, desire):
		"""
		Add new desire to resolver

		:param desire: a desire to be added to the resolver
		:type desire: Desire
		:return: returns nothing
		"""
		if isinstance(desire, DesCmdVel):
			self.lst_desires_vel.append(desire)
		elif isinstance(desire, DesTransVel):
			self.lst_des_transvel.append(desire)
		elif isinstance(desire, DesTransDir):
			self.lst_des_transdir.append(desire)
		elif isinstance(desire, DesRotVel):
			self.lst_des_rotvel.append(desire)
		else:
			print("Type of desire not known: " + type(desire).__name__)


	def reset(self):
		"""
		Reset all lists (desires and behaviours)

		"""
		self.lst_behaviours = []
		self.lst_desires_vel = []
		self.lst_des_transvel =  []
		self.lst_des_rotvel =  []
		self.lst_des_transdir =  []


	def resetDesires(self):
		"""
		Reset all desire lists

		"""
		self.lst_desires_vel = []
		self.lst_des_transvel = []
		self.lst_des_rotvel = []
		self.lst_des_transdir = []


	def set_behaviour_lst(self, behaviours):
		"""
		set the new active behaviour list

		:param lst_behaviour: list of behaviour which are currently active
		:type lst_behaviour: list(Behaviour)
		"""
		self.lst_behaviours = behaviours


	def resolveDesire(self, lst_desires):
		"""
		function to resolve a value from a list of similar desires 
		
		:param lst_desires: a list of desires (all items of the same type)
		:type lst_desires: list[Desire]
		:return: returns the resolved value (depending on type of value in desire, can also be an array)
		"""

		output_value = 0

		if len(lst_desires) > 0:

			# sort list of desires based on priority (high to low)
			lst_desires.sort(key=lambda x: x.priority, reverse=True)

			# print type of desires and whole list
			#print("Desire-Type: " + type(lst_desires[0]).__name__)
			#for d in lst_desires:
			#	print(d)

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

		return output_value

	def runOnce(self):
		"""
		Run one cycle of the resolver to calculate output values
		and publish corresponding messages

		"""

		# clear all desire lists
		self.resetDesires()

		# trigger the fire()-function of all active behaviours
		for beh in self.lst_behaviours:
			beh.fire()

		# resolve desires to get velocity values
		transvel = self.resolveDesire(self.lst_des_transvel)
		transdir = self.resolveDesire(self.lst_des_transdir)
		rotvel = self.resolveDesire(self.lst_des_rotvel)

		# create cmd_vel msg out of resolved values
		cmd_msg = Twist()
		cmd_msg.linear.x = transvel * math.cos(math.radians(transdir))
		cmd_msg.linear.y = transvel * math.sin(math.radians(transdir))
		cmd_msg.linear.z = 0.0
		cmd_msg.angular.x = 0.0
		cmd_msg.angular.y = 0.0
		cmd_msg.angular.z = rotvel

		# send update (limit sending speed to 1msg/sec if no change occurred)
		rospy.loginfo("Resolver: trans_vel=%f, rot_vel=%f", transvel, rotvel)
		self.moveactuator.send_twist(cmd_msg)
