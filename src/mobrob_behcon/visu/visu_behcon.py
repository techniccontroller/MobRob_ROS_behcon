from graphics import *

class VisuBehCon:
	"""
	The class VisuBehCon

	This class visualizes the current software configuration of the behaviour based control of the robot. 
	This includes the strategy and all behaviours with their priorities. 
	
	During runtime, it will also update the current state of strategy and the output to the resolver.
	"""

	def __init__(self, behconnode):
		"""
		constructor

		:param behconnode: the BehConNode to be visulatized
		:type behconnode: BehConNode
		"""

		self.behconnode = behconnode

		self.clr_lvl1 = "#fff2cc"
		self.clr_lvl2 = "#f8cbad"
		self.clr_lvl3 = "#ffffff"
		self.clr_lvl4 = "#d6dce5"
		self.clr_lvl5 = "#e2f0d9"
		self.clr_lvl6 = "#ffe699"
		self.clr_lvl7 = "#ffff9f"
		self.clr_lvl8 = "#bdd7ee"
		self.clr_lvl9 = "#deebf7"

		self.rect_behGroups = []
		self.lbl_behGroups = []
		self.ovl_behGroups = []


	def create_box_w_lbl(self, pt, width, height, color, text):
		"""
		Creates a box with text

		:param pt: position of rectangle (left top corner)
		:type pt: (int, int)
		:param width: width of rectangle
		:type width: int
		:param height: height of rectangle
		:type height: int
		:param color: color of rectangle
		:type color: string
		:param text: text
		:type text: string
		:return: tkinter rectangle and text object
		:rtype: Rectangle, Text
		"""
		rect = Rectangle(pt, Point(pt.x+width, pt.y+height))
		rect.setFill(color)
		rect.setOutline("black")
		
		lbl = Text(Point(pt.x+5, pt.y+2), text)
		lbl.setSize(9)
		lbl.config["anchor"] = tk.NW
		lbl.config["justify"] = tk.LEFT
		lbl.setStyle("bold")
		lbl.setFace("courier")

		return rect, lbl

	
	def draw(self):
		"""
		This function draws the whole configuration in a tkinter window.
		"""
		# get number of behaviour groups
		num_beh_group = len(self.behconnode.strategy.lst_behgrps)
		print(num_beh_group)

		# define parameters
		border = 170
		margin = 5
		w_per_grp = 200
		win_width = 2*border+w_per_grp*num_beh_group
		win_height = 500

		y_stra = 50
		x_stra = border
		w_stra = win_width-2*x_stra
		h_stra = 100

		y_behgrp = 165
		w_behgrp = w_per_grp-2*margin
		h_behgrp = 290

		y_per = 50
		x_per = 20
		w_per = 130
		h_per = win_height - 70

		y_res = 50
		x_res = win_width - 150
		w_res = 130
		h_res = win_height - 70

		# create window
		self.win = GraphWin('BehCon', win_width, win_height)

		# draw behconnode
		rct_behcon, lbl_behcon = self.create_box_w_lbl(Point(10, 10), win_width-20, win_height-20, self.clr_lvl1, "BehConNode")
		rct_behcon.draw(self.win)
		lbl_behcon.draw(self.win)


		# draw strategy
		rct_strgy, lbl_strgy = self.create_box_w_lbl(Point(x_stra, y_stra), w_stra, h_stra, self.clr_lvl2, "Strategy")
		rct_strgy.draw(self.win)
		lbl_strgy.draw(self.win)
		
		self.ovl_behGroups = []

		# draw each behaviour group
		i = 0
		for beh_group in self.behconnode.strategy.lst_behgrps:
			x_behgrp = border+5+w_per_grp*i
			oval = Oval(Point(x_behgrp+w_behgrp/2-20, 85), Point(x_behgrp+w_behgrp/2+20, 115))
			if beh_group.active == True:
				oval.setFill("green")
			else:
				oval.setFill(self.clr_lvl3)
			oval.draw(self.win)

			ovaltext = Text(Point(x_behgrp+w_behgrp/2, 100), beh_group.success)
			ovaltext.draw(self.win)
			
			rct_behgrp, lbl_behgrp = self.create_box_w_lbl(Point(x_behgrp, y_behgrp), w_behgrp, h_behgrp, self.clr_lvl4, beh_group.name)
			rct_behgrp.draw(self.win)
			lbl_behgrp.draw(self.win)
			line = Line(Point(x_behgrp+w_behgrp/2, y_behgrp-50), Point(x_behgrp+w_behgrp/2, y_behgrp))
			#line.setArrow("last")
			line.draw(self.win)
			
			if i != 0:
				line2 = Line(Point(x_behgrp+w_behgrp/2-w_per_grp+20, y_behgrp-65), Point(x_behgrp+w_behgrp*0.5-20, y_behgrp-65))
				line2.setArrow("last")
				line2.draw(self.win)
			i = i+1

			y_beh = y_behgrp + 20
			h_beh = 40
			if len(beh_group.lst_behaviours) > 6:
				h_beh = 30

			# draw the behaviours of the behaviour group
			for beh in beh_group.lst_behaviours:
				rct_beh, lbl_beh = self.create_box_w_lbl(Point(x_behgrp+margin, y_beh), w_behgrp-2*margin, h_beh, self.clr_lvl5, beh.behaviour.name)
				rct_beh.draw(self.win)
				lbl_beh.draw(self.win)
				text = Text(Point(x_behgrp+margin+w_behgrp-2*margin-30, y_beh+3*margin), beh.priority)
				text.draw(self.win)
				y_beh = y_beh + h_beh + margin

			# save handles to oval graphic element of behaviour group to update it later
			self.ovl_behGroups.append([oval, beh_group, ovaltext])
		
		h_beh = 40

		# draw perceptual space (sensors)
		rct_per, lbl_per = self.create_box_w_lbl(Point(x_per, y_per), w_per, h_per, self.clr_lvl6, "PerceptualSpace")
		rct_per.draw(self.win)
		lbl_per.draw(self.win)

		y_per = y_per + 20

		if self.behconnode.percept_space.laserscanner:
			rct, lbl = self.create_box_w_lbl(Point(x_per+margin, y_per), w_per-2*margin, h_beh, self.clr_lvl7, "LaserScanner")
			rct.draw(self.win)
			lbl.draw(self.win)
			lbl2 = Text(Point(x_per+margin+5, y_per+20), self.behconnode.percept_space.laserscanner.ros_topic)
			lbl2.setSize(9)
			lbl2.config["anchor"] = tk.NW
			lbl2.config["justify"] = tk.LEFT
			lbl2.setFace("courier")
			lbl2.draw(self.win)
			y_per = y_per + h_beh + margin
		if self.behconnode.percept_space.camera:
			rct, lbl = self.create_box_w_lbl(Point(x_per+margin, y_per), w_per-2*margin, h_beh, self.clr_lvl7, "Camera")
			rct.draw(self.win)
			lbl.draw(self.win)
			lbl2 = Text(Point(x_per+margin+5, y_per+20), self.behconnode.percept_space.camera.port)
			lbl2.setSize(9)
			lbl2.config["anchor"] = tk.NW
			lbl2.config["justify"] = tk.LEFT
			lbl2.setFace("courier")
			lbl2.draw(self.win)
			y_per = y_per + h_beh + margin
		if self.behconnode.percept_space.egopose:
			rct, lbl = self.create_box_w_lbl(Point(x_per+margin, y_per), w_per-2*margin, h_beh, self.clr_lvl7, "EgoPose")
			rct.draw(self.win)
			lbl.draw(self.win)
			lbl2 = Text(Point(x_per+margin+5, y_per+20), self.behconnode.percept_space.egopose.ros_topic)
			lbl2.setSize(9)
			lbl2.config["anchor"] = tk.NW
			lbl2.config["justify"] = tk.LEFT
			lbl2.setFace("courier")
			lbl2.draw(self.win)
			y_per = y_per + h_beh + margin

		# draw resolver (actuators)
		rct_res, lbl_res = self.create_box_w_lbl(Point(x_res, y_res), w_res, h_res, self.clr_lvl8, "Resolver")
		rct_res.draw(self.win)
		lbl_res.draw(self.win)

		y_res = y_res + 20

		num_destransvel = len(self.behconnode.resolver.lst_des_transvel)
		rct, self.lbl_destransvel = self.create_box_w_lbl(Point(x_res+margin, y_res), w_res-2*margin, h_beh, self.clr_lvl9, "DesTransVel[" + str(num_destransvel) + "]")
		rct.draw(self.win)
		self.lbl_destransvel.draw(self.win)
		y_res = y_res + h_beh + margin
		
		num_destransdir = len(self.behconnode.resolver.lst_des_transdir)
		rct, self.lbl_destransdir = self.create_box_w_lbl(Point(x_res+margin, y_res), w_res-2*margin, h_beh, self.clr_lvl9, "DesTransDir[" + str(num_destransdir) + "]")
		rct.draw(self.win)
		self.lbl_destransdir.draw(self.win)
		y_res = y_res + h_beh + margin
		
		num_desrotvel = len(self.behconnode.resolver.lst_des_rotvel)
		rct, self.lbl_desrotvel = self.create_box_w_lbl(Point(x_res+margin, y_res), w_res-2*margin, h_beh, self.clr_lvl9, "DesRotVel[" + str(num_desrotvel) + "]")
		rct.draw(self.win)
		self.lbl_desrotvel.draw(self.win)
		y_res = y_res + h_beh + margin

	def update(self):
		"""
		This function will update the state of Strategy and number of Desires in Resolver.
		"""
		# update active behaviour group
		for item in self.ovl_behGroups:
			if item[1].active == True:
				item[0].setFill("green")
			else:
				item[0].setFill(self.clr_lvl3)
			item[2].setText(item[1].success)

		# update number of desires
		num_destransvel = len(self.behconnode.resolver.lst_des_transvel)
		self.lbl_destransvel.setText("DesTransVel[" + str(num_destransvel) + "]")
		num_destransdir = len(self.behconnode.resolver.lst_des_transdir)
		self.lbl_destransdir.setText("DesTransDir[" + str(num_destransdir) + "]")
		num_desrotvel = len(self.behconnode.resolver.lst_des_rotvel)
		self.lbl_desrotvel.setText("DesRotVel[" + str(num_desrotvel) + "]")

