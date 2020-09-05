#!/usr/bin/env python

from mobrob_behcon.desires.desire import Desire

class DesCmdVel(Desire):
    """
    The class DesCmdVel

    Represents a Desire for controlling the movement of the robot.
    Similar to its parent class it hase three attributes: value, strength, priority.
    The value is a tuple of 6 float values mapped to following values 
    in **/cmd_vel** message: [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
    """

    def __init__(self, value, strength):
        super(DesCmdVel, self).__init__(value, strength, function=None)


# class DesCmdGripper(Desire):
#     """
#     The class DesCmdGripper

#     Represents a Desire for controlling the gripper of the robot.
#     There are two spcialities about this type of desire.

#     - all desires of this type will be executed ignoreing **strength** and **priority**.
    
#     - there is a aditional attribute called **function** which holds a reference to a function to be executed by resolver. 
#     As parameter the attribute **value** will be inserted.


#     """

#     def __init__(self, value=0, strength=1.0, function=None):
#         super(DesCmdGripper, self).__init__(value, strength, function)


class DesCmdLight(Desire):

    def __init__(self, value, strength):
        super(DesCmdLight, self).__init__(value, strength, function=None)