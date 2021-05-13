.. MobRob_ROS_behcon documentation master file, created by
   sphinx-quickstart on Sat Aug 22 21:55:07 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to **MobRob_ROS_behcon**'s documentation!
=================================================

Modules:

.. toctree::
   :maxdepth: 4



:mod:`mobrob_behcon.core` --- core module
-----------------------------------------

This module contains core classes. 

.. currentmodule:: mobrob_behcon.core

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   behcon_node.BehConNode
   resolver.Resolver
   perceptual_space.PerceptualSpace
   visualisation.KOOSVisu


:mod:`mobrob_behcon.desires` --- desires module
-----------------------------------------------

This module contains several classes about desires. 

.. currentmodule:: mobrob_behcon.desires

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   desire.Desire
   desires.DesCmdVel
   desires.DesTransVel
   desires.DesRotVel
   desires.DesCmdLight



:mod:`mobrob_behcon.connectors` --- connectors module
-----------------------------------------------------

This module contains classes to connect sensors and actuators. 

.. currentmodule:: mobrob_behcon.connectors

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   camera.CameraTCP
   egopose.EgoPose
   laserscanner.LaserScanner
   gripper.Gripper


:mod:`mobrob_behcon.strategies` --- strategies module
-----------------------------------------------------

This module contains strategy classes. 

.. currentmodule:: mobrob_behcon.strategies

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   strategy.Strategy
   rd_strategy.RDStrategy

:mod:`mobrob_behcon.behaviours` --- behaviours module
-----------------------------------------------------

This module contains classes about behaviours. 

.. currentmodule:: mobrob_behcon.behaviours

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   behaviour.Behaviour
   beh_align.BehAlign
   beh_limfor.BehLimFor
   beh_stop.BehStop
   beh_reached.BehReached
   beh_transvel.BehConstTransVel
   beh_turn.BehTurn
   behaviourgroup.BehaviourGroup

:mod:`mobrob_behcon.visu` --- visu module
-----------------------------------------------------

This module contains classes about behaviours. 

.. currentmodule:: mobrob_behcon.visu

Class overview:

.. autosummary::
   :toctree: stubs
   :template: class.rst

   visu_behcon.VisuBehCon
   graphics.GraphWin
   Transform
   


:mod:`mobrob_behcon.utils` --- utils module
-----------------------------------------------------

This module contains classes with several helpful utils. 

.. currentmodule:: mobrob_behcon.utils

Function overview:

.. autosummary::
   :toctree: stubs
   :template: base.rst

   mobrob_transformation.get_T_world_robot
   mobrob_transformation.get_T_robot_laser
   mobrob_transformation.get_T_laser_cluster
   mobrob_transformation.get_T_of_3Dpoint
   mobrob_transformation.get_T_world_cluster
   mobrob_transformation.get_world_coordinate
   mobrob_transformation.get_laser_coordinate
   mobrob_transformation.get_robot_coordinate
   mobrob_transformation.get_robot_corners


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

