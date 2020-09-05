#!/usr/bin/env python

import rospy
from myrobot_model.srv import AttinyCommand

class Gripper(object):
    """
    The class Gripper

    Represents a connector to the gripper service of the robot.
    Provides multiple functions to control the gripper.
    """

    def __init__(self, ros_service):
        """
        constructor

        :param ros_service: possible topics are **'/pose'** (myrobot_model/Pose) or **'/odom'** (nav_msgs/Odometry)
        :type ros_service: string
        """
        rospy.wait_for_service('attiny_command')
        self.attiny_command = rospy.ServiceProxy('attiny_command', AttinyCommand)
        
    
    def send_command(self, command):
        try:
            resp = self.attiny_command(command)
            return resp.output
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    

    def setActivServo(self, value):
        print(self.send_command("sv_ac(" + ("1" if value else "0") + ")"))
    
    
    def writeServo(self, angle):
        print(self.send_command("sv_wr(" + str(angle) + ")\n"))
    

    def refreshServo(self):
        print(self.send_command("sv_rf(1)\n"))
    

    def initGRIP(self):
        print(self.send_command("gr_it(1)\n"))
    

    def initVERT(self):
        print(self.send_command("vt_it(1)\n"))
    

    def setSpeedGRIP(self, value):
        print(self.send_command("gr_sp(" + str(abs(value)) + ")\n"))
    

    def setSpeedVERT(self, value):
        print(self.send_command("vt_sp(" + str(abs(value)) + ")\n"))
    

    def moveAbsGRIP(self, value):
        print(self.send_command("gr_ma(" + str(value) + ")\n"))
    

    def moveAbsVERT(self, value):
        print(self.send_command("vt_ma(" + str(value) + ")\n"))
    

    def moveRelGRIP(self, value):
        print(self.send_command("gr_mr(" + str(value) + ")\n"))
    

    def moveRelVERT(self, value):
        print(self.send_command("vt_mr(" + str(value) + ")\n"))
    

    def stopGRIP(self):
        print(self.send_command("gr_st(1)\n"))
    

    def stopVERT(self):
        print(self.send_command("vt_st(1)\n"))
    

    def stopAll(self):
        print(self.send_command("st_st(1)\n"))
    

    def setStayActivStepper(self, value):
        print(self.send_command("st_ac(" + ("1" if value else "0") + ")\n"))
    

    def getPosVERT(self):
        print(self.send_command("vt_gp(1)\n"))
    

    def getPosGRIP(self):
        print(self.send_command("gr_gp(1)\n"))
    