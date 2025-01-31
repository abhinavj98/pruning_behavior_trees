# Pruning Behavior Trees



## Overview



This repository focuses on utilizing **Behavior Trees (BTs) in ROS2** to manage a reinforcement learning-based controller for autonomous tree pruning. Behavior trees provide a modular and interpretable framework for designing robotic control policies, making it easier to integrate learning-based and rule-based approaches.



The system leverages **reinforcement learning (RL)** for visuomotor control, allowing a UR5 robotic arm to identify and prune branches effectively.



## Key Features



- **Behavior Tree Framework**: Modular and hierarchical structure for decision-making.

- **ROS2 Integration**: Seamless communication between robot components using ROS py_trees.

- **Joystick Interface**: Maps robot commands to joystick to enable quick experimentation in the field.

- **GUI Integration**: RVIZ-based GUI to select pruning points given a point cloud of the tree.





*Under construction*

Use behavior trees to run a reinforcement learning controller for the task of pruning. Behavior trees allow modularity while building large robotics systems and ease of integration.

sudo rmmod nvidia_uvm
sudo modprobe nvidia_uvm
