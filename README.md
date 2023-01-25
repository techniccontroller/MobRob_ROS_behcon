# MobRob_ROS_behcon
ROS-Package: **mobrob_behcon** node for behaviour-based control of [MobRob](https://techniccontroller.de/mobrob-ros-software-architecture/)

![alt text](https://techniccontroller.de/wp-content/uploads/mobrob_behcon_node_2.png "behaviour based control")

## Short description of behavior-based control concept

Behavior-based control was already described in 1997 by Kurt Konolige and Karen Myers as Saphire architecture. The idea is to have elementary behavioral patterns that form the foundation stones for intelligence. For example:

- Food intake
- Obstacle avoidance
- Exploratory behavior

The behavioral patterns (behaviors) are independent of each other and can therefore be applied in any number and sequence-specific to the situation. The behavioral patterns calculate desired outputs (desires) for the actuators. The so-called resolver then combines the desires into an absolute command for the actuators.

Depending on the order of the mobile robot, the processing status, and the robot environment, the behavior patterns are activated and deactivated at a strategic level. With today's application complexity, the strategic level can be realized by a finite state automaton.

More details are on my Website: https://techniccontroller.de/mobrob-behaviour-based-control/

## Folders

**doc**: contains sphinx documentation of all classes ([latest html deployment](https://techniccontroller.github.io/MobRob_ROS_behcon/))

**nodes**: contains the main files, for different tasks to be executed

**src**: contains all source codes, seperated in different domains
