exploration_ros
=========

A ROS package that implements a robot exploration system.
Actually only the single robot part has been developed.

Requirements:
-------------

- **Eigen3** 
 
- The **srrg_core** package developed by G. Grisetti provides libraries and utilities.

- [ROS indigo.](http://wiki.ros.org/indigo/Installation) or [ROS Kinetic.](http://wiki.ros.org/kinetic/Installation)

- The **ROS Navigation stack** 

        $ git clone https://github.com/ros-planning/navigation.git




- (OPTIONAL) This code uses **cg_mrslam** developed by M.T. Lazaro for implementing SLAM techniques 

        $ git clone https://github.com/mtlazaro/cg_mrslam.git


- (OPTIONAL) To run the simulation example it's necessary **ROS Stage**

        $ git clone https://github.com/ros-simulation/stage_ros


Installation
------------
The code has been tested on Ubuntu 14.04 and 16.04 (64bits). 

- Download the source code to your ROS workspace directory

- ROS (catkin):
  - Installation for ROS is supported
  - In your catkin workspace 

            $ catkin_make 

Instructions
------------

#### Nodes

- **frontier_planner:**
 This node, given an occupancy map and a pose of the robot, generates and send goals to the move_base action client.

The default parameters work well in general, so it can be sufficient to run the node without specifying any of them. In the launch file launch/stage_experiment.launch you can see a list of available parameters.
  
#### Example of use

It's possible to test the whole system by using a launch file present in the package. 

    $ roslaunch exploration_ros stage_experiment.launch

#### Integrating the system with other components

The frontier planner node can be used also with different SLAM nodes, in particular it has been already tested with gmapping. The only requirements are the availability of an occupancy grid map, a cost map and of a map -> robot TF transform.


