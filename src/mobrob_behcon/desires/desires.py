#!/usr/bin/env python

from mobrob_behcon.desires.desire import Desire

class DesCmdVel(Desire):
    """
    The class DesCmdVel

    Represents a Desire for controlling the movement of the robot.
    Similar to its parent class it hase three attributes: value, strength, priority.
    value is a tuple of 6 float values mapped to following values 
    in **/cmd_vel** message: [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
    """

    def __init__(self, value, strength):
        super(DesCmdVel, self).__init__(value, strength)


class DesCmdGripper(Desire):

    def __init__(self, value, strength):
        super(DesCmdGripper, self).__init__(value, strength)


class DesCmdLight(Desire):

    def __init__(self, value, strength):
        super(DesCmdLight, self).__init__(value, strength)