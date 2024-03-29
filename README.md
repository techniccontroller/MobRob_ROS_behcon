# MobRob_ROS_behcon
ROS-Package: **mobrob_behcon** node for behaviour-based control of [MobRob](https://techniccontroller.com/mobrob-ros-software-architecture/)

![alt text](https://techniccontroller.com/wp-content/uploads/mobrob_behcon_node_2.png "behaviour based control")

## Short description of behavior-based control concept

Behavior-based control was already described in 1997 by Kurt Konolige and Karen Myers as Saphira architecture. The idea is to have elementary behavioral patterns that form the foundation stones for intelligence. For example:

- Food intake
- Obstacle avoidance
- Exploratory behavior

The behavioral patterns (behaviors) are independent of each other and can therefore be applied in any number and sequence specific to the situation. The behavioral patterns calculate desired outputs (desires) for the actuators. The so-called resolver then combines the desires into an absolute command for the actuators.

Depending on the order of the mobile robot, the processing status, and the robot environment, the behavior patterns are activated and deactivated at a strategic level. With today's application complexity, the strategic level can be realized by a finite state automaton.

More details are on my Website: https://techniccontroller.com/mobrob-behaviour-based-control/

## Folders

**doc**: contains sphinx documentation of all classes ([latest html deployment](https://techniccontroller.github.io/MobRob_ROS_behcon/))

**nodes**: contains the main files, for different tasks to be executed

**src**: contains all source codes, separated in different domains


## References

- Konolige, Kurt & Myers, Karen & Ruspini, Enrique & Saffiotti, Alessandro. (1997). The Saphira Architecture: A Design for Autonomy. Journal of Experimental & Theoretical Artificial Intelligence. 9. 10.1080/095281397147095. 

  https://www.researchgate.net/publication/2429137_The_Saphira_Architecture_A_Design_for_Autonomy


- Konolige, Kurt & Myers, Karen. (1996). The Saphira Architecture for Autonomous Mobile Robots. 

  https://courses.cs.washington.edu/courses/cse571/00au/papers/saphira-konolige.pdf
