# -*- coding: utf-8 -*-
import rospy
import os
from tkinter import *
import tkinter.font as font

from geometry_msgs.msg import Twist

from mobrob_behcon.connectors.moveactuator import MoveActuator
from mobrob_behcon.connectors.gripper import Gripper


class ManualControlApp(Tk, object):

    def __init__(self):
        super(ManualControlApp, self).__init__()
        # Set speeds for control
        self.trans_vel = 0.2

        self.move_bindings = {
            'z': [1, 0, 0, 0],
            'u': [1, 0, 0, -1],
            'g': [0, 0, 0, 1],
            'j': [0, 0, 0, -1],
            't': [1, 0, 0, 1],
            'n': [-1, 0, 0, 0],
            'm': [-1, 0, 0, 1],
            'b': [-1, 0, 0, -1],
            'U': [1, -1, 0, 0],
            'Z': [1, 0, 0, 0],
            'G': [0, 1, 0, 0],
            'J': [0, -1, 0, 0],
            'T': [1, 1, 0, 0],
            'N': [-1, 0, 0, 0],
            'M': [-1, -1, 0, 0],
            'B': [-1, 1, 0, 0],
            'h': [0, 0, 0, 0],
            'H': [0, 0, 0, 0]
        }

        self.move_cmds = [['🡼\nt', '🡹\nz', '🡽\nu'],
                          ['🡸\ng', '●\nh', '🡺\nj'],
                          ['🡿\nb', '🡻\nn', '🡾\nm']]

        # Initialize robot in default state
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.linear.y = 0.0
        self.cmd_msg.linear.z = 0.0
        self.cmd_msg.angular.x = 0.0
        self.cmd_msg.angular.y = 0.0
        self.cmd_msg.angular.z = 0.0
        self.moving = False

        # Initialize the teleop node
        rospy.init_node("teleop_node")
        self.move_actuator = MoveActuator()
        self.gripper = Gripper()

        # Set up the interface
        self.grid_rowconfigure(0, weight=1)  # this needed to be added
        self.grid_columnconfigure(0, weight=1)  # as did this
        self.title('ManualControlApp')
        self.geometry("880x550")
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)

        self.pixelVirtual = PhotoImage(width=1, height=1)
        self.myFont = font.Font(family='Helvetica', size=20, weight='bold')
        self.myFontSmall = font.Font(family='Helvetica', size=10, weight='bold')

        vcmd = (self.register(self.validate),
                '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')

        # -----------------------------------------------------------------------------------------------------------
        # Create Frame widget for move control
        move_act_frame = Frame(self, width=350, height=600, highlightbackground="gray", highlightthickness=1)
        move_act_frame.grid(row=0, column=0, padx=10, pady=10, sticky=N)

        Label(move_act_frame, text="Movement Control").grid(row=1, column=0, columnspan=2, padx=5, pady=5)

        # Create frame for buttons
        btns_frame = Frame(move_act_frame, width=300, height=300, bg="purple")
        btns_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

        self.btn_lbls = []
        for r in range(3):
            for c in range(3):
                btn_lbl = StringVar()
                btn_lbl.set(self.move_cmds[r][c])
                btn = Button(btns_frame, textvariable=btn_lbl, image=self.pixelVirtual, height=100, width=100,
                             compound='center')
                btn.bind('<ButtonPress-1>', self.btn_down)
                btn.bind('<ButtonRelease-1>', self.btn_up)
                btn['font'] = self.myFont
                btn.grid(row=r + 1, column=c)
                self.btn_lbls.append(btn_lbl)

        Label(move_act_frame, text="Speed: ").grid(row=3, sticky=W, padx=5, pady=5)
        self.entry_spd = Entry(move_act_frame, width=4, validate='key', validatecommand=vcmd)
        self.entry_spd.insert(0, '0.2')
        self.entry_spd.grid(row=3, column=1, sticky=W, padx=5)

        # -----------------------------------------------------------------------------------------------------------
        # Create Frame widget for gripper control
        grip_frame = Frame(self, width=350, height=500, highlightbackground="gray", highlightthickness=1)
        grip_frame.grid(row=0, column=1, sticky=N, padx=10, pady=10)

        Label(grip_frame, text="Gripper Control").grid(row=1, column=0, columnspan=2, padx=5, pady=5)

        # Create frame for buttons
        btns_frame = Frame(grip_frame, width=300, height=300)
        btns_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

        # btn = Button(btns_frame, text='INIT VERT')
        # btn.bind('<ButtonPress-1>', lambda e: self.gripper.initVERT())
        # btn.bind('<ButtonRelease-1>', None)
        # btn['font'] = self.myFontSmall
        # btn.grid(row=0, column=0)
        # btn = Button(btns_frame, text='INIT GRIP')
        # btn.bind('<ButtonPress-1>', lambda e: self.gripper.initGRIP())
        # btn.bind('<ButtonRelease-1>', None)
        # btn['font'] = self.myFontSmall
        # btn.grid(row=0, column=1)

        btnInitGRIP = Button(btns_frame, text="Init GRIP")
        btnInitGRIP.bind('<ButtonPress-1>', lambda e: self.gripper.initGRIP())
        self.preset_button(btnInitGRIP, 0, 0)

        btnInitVERT = Button(btns_frame, text="Init VERT")
        btnInitVERT.bind('<ButtonPress-1>', lambda e: self.gripper.initVERT())
        self.preset_button(btnInitVERT, 1, 0)

        # Vertical
        btnUp = Button(btns_frame, text="Up")
        btnUp.bind('<ButtonPress-1>', lambda e: self.control_up())
        btnUp.bind('<ButtonRelease-1>', lambda e: self.control_stop())
        self.preset_button(btnUp, 0, 1)

        btnDown = Button(btns_frame, text="Down")
        btnDown.bind('<ButtonPress-1>', lambda e: self.control_down())
        btnDown.bind('<ButtonRelease-1>', lambda e: self.control_stop())
        self.preset_button(btnDown, 1, 1)

        btnMoveToVERT = Button(btns_frame, text="Set")
        btnMoveToVERT.bind('<ButtonPress-1>', lambda e: self.control_vert_abs())
        self.preset_button(btnMoveToVERT, 2, 1)

        self.txtPosVERT = Entry(btns_frame, width=5, validate='key', validatecommand=vcmd)
        self.txtPosVERT.insert(0, '0')
        self.txtPosVERT.grid(column=3, row=1)

        btnGetPosVERT = Button(btns_frame, text="getPos")
        btnGetPosVERT.bind('<ButtonPress-1>', lambda e: self.get_pos_vert())
        self.preset_button(btnGetPosVERT, 4, 1)

        # Gripper
        btnOpen = Button(btns_frame, text="Open")
        btnOpen.bind('<ButtonPress-1>', lambda e: self.control_open())
        btnOpen.bind('<ButtonRelease-1>', lambda e: self.control_stop())
        self.preset_button(btnOpen, 0, 2)

        btnClose = Button(btns_frame, text="Close")
        btnClose.bind('<ButtonPress-1>', lambda e: self.control_close())
        btnClose.bind('<ButtonRelease-1>', lambda e: self.control_stop())
        self.preset_button(btnClose, 1, 2)

        btnMoveToGRIP = Button(btns_frame, text="Set")
        btnMoveToGRIP.bind('<ButtonPress-1>', lambda e: self.control_grip_abs())
        self.preset_button(btnMoveToGRIP, 2, 2)

        self.txtPosGRIP = Entry(btns_frame, width=5, validate='key', validatecommand=vcmd)
        self.txtPosGRIP.insert(0, '0')
        self.txtPosGRIP.grid(column=3, row=2)

        btnGetPosGRIP = Button(btns_frame, text="getPos")
        btnGetPosGRIP.bind('<ButtonPress-1>', lambda e: self.get_pos_grip())
        self.preset_button(btnGetPosGRIP, 4, 2)

        btnActive = Button(btns_frame, text="activate")
        btnActive.bind('<ButtonPress-1>', lambda e: self.gripper.setStayActivStepper(True))
        self.preset_button(btnActive, 0, 3)

        btnDeactivate = Button(btns_frame, text="deactivate")
        btnDeactivate.bind('<ButtonPress-1>', lambda e: self.gripper.setStayActivStepper(False))
        self.preset_button(btnDeactivate, 1, 3)

        btnServoActive = Button(btns_frame, text="deactivate\nServo")
        btnServoActive.bind('<ButtonPress-1>', lambda e: self.gripper.setActivServo(False))
        self.preset_button(btnServoActive, 0, 4)

        btnServoWrite = Button(btns_frame, text="Servo\nwrite")
        btnServoWrite.bind('<ButtonPress-1>', lambda e: self.write_servo())
        self.preset_button(btnServoWrite, 1, 4)

        self.txtPosServo = Entry(btns_frame, width=4, validate='key', validatecommand=vcmd)
        self.txtPosServo.insert(0, '50')
        self.txtPosServo.grid(column=2, row=4)

        btnServoRefresh = Button(btns_frame, text="Servo\nrefresh")
        btnServoRefresh.bind('<ButtonPress-1>', lambda e: self.gripper.refreshServo())
        self.preset_button(btnServoRefresh, 3, 4)

        self.mainloop()

    def preset_button(self, btn, col, row):
        btn.config(image=self.pixelVirtual, width=55, height=55, compound='center')
        btn['font'] = self.myFontSmall
        btn.grid(row=row, column=col)

    def keydown(self, event):
        print("Key pressed: ", event.keysym)
        self.send_command(event.keysym)

    def send_command(self, cmd):
        if cmd in self.move_bindings.keys():
            speed = float(self.entry_spd.get())
            dirArray = self.move_bindings[cmd]
            x = dirArray[0] * speed
            y = dirArray[1] * speed
            th = dirArray[3] * speed
            self.move_actuator.send_vel_2d(x, y, th)
        elif cmd == "Shift_L":
            for btn_lbl in self.btn_lbls:
                btn_lbl.set(btn_lbl.get().upper())

    def keyup(self, event):
        if event.keysym == "Shift_L":
            for btn_lbl in self.btn_lbls:
                btn_lbl.set(btn_lbl.get().lower())
        else:
            self.move_actuator.send_vel_2d(0, 0, 0)

    def btn_down(self, event):
        self.send_command(event.widget['text'][-1])

    def btn_up(self, event):
        self.move_actuator.send_vel_2d(0, 0, 0)

    def validate(self, action, index, value_if_allowed,
                 prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed:
            try:
                float(value_if_allowed)
                return True
            except ValueError:
                return False
        elif value_if_allowed == "":
            return True
        else:
            return False

    def control_up(self):
        self.gripper.moveRelVERT(-900)

    def control_down(self):
        self.gripper.moveRelVERT(900)

    def control_open(self):
        self.gripper.moveRelGRIP(900)

    def control_close(self):
        self.gripper.moveRelGRIP(-900)

    def control_grip_abs(self):
        self.gripper.moveAbsGRIP(int(self.txtPosGRIP.get()))

    def control_vert_abs(self):
        self.gripper.moveAbsVERT(int(self.txtPosVERT.get()))

    def active_stepper(self):
        self.gripper.setStayActivStepper(True)

    def deactivate_stepper(self):
        self.gripper.setStayActivStepper(False)

    def write_servo(self):
        self.gripper.writeServo(int(self.txtPosServo.get()))

    def control_stop(self):
        self.gripper.stopAll()

    def get_pos_vert(self):
        pos = self.gripper.getPosVERT()
        self.txtPosVERT.delete(0, END)
        self.txtPosVERT.insert(0, str(int(pos)))

    def get_pos_grip(self):
        pos = self.gripper.getPosGRIP()
        self.txtPosGRIP.delete(0, END)
        self.txtPosGRIP.insert(0, str(int(pos)))


if __name__ == '__main__':
    os.system('xset r off')
    control = ManualControlApp()
    os.system('xset r on')
