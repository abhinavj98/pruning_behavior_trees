# Pruning Behavior Trees

## Overview

This repository focuses on utilizing **Behavior Trees (BTs) in ROS2** to manage a reinforcement learning-based controller for autonomous tree pruning. Behavior trees provide a modular and interpretable framework for designing robotic control policies, making it easier to integrate learning-based and rule-based approaches.

The system leverages **reinforcement learning (RL)** for visuomotor control, allowing a UR5 robotic arm to identify and prune branches effectively.

## Key Features

- **Behavior Tree Framework**: Modular and hierarchical structure for decision-making.
- **ROS2 Integration**: Seamless communication between robot components using ROS `py_trees_ros`.
- **Joystick Interface**: Maps robot commands to joystick to enable quick experimentation in the field.
- **GUI Integration**: RVIZ-based GUI to select pruning points given a point cloud of the tree.

## Setup and Installation

1. Ensure you have ROS 2 installed (e.g., Humble or Foxy).
2. Install `py_trees_ros` and required dependencies.

```bash 
sudo apt update
sudo apt install ros-humble-py-trees-ros
```
3. Ensure you have the `pruning_sb3` environment set up for reinforcement learning models. Since `pruning_sb3` is a git submodule, you will need to clone it by running:
```bash
git submodule update --init --recursive
```

## Launching the Behavior Tree

The primary entry point for launching the behavior tree system is the `run_pruning_bt.py` script. It initializes the required ROS 2 nodes, sets up the reinforcement learning behavior tree, and executes the IO processing tree.

To run the behavior tree, execute the script with the required arguments:

```bash
python3 run_pruning_bt.py --tree_type <TYPE> --trial_number <TRIAL> --tree_number <NUMBER> [--rotated]
```

### Arguments

- `--tree_type` (str): Type of the tree being operated on (e.g., `'pruner'`, `'test'`). Required.
- `--trial_number` (int): An integer representing the trial number (e.g., `1`, `2`, `3`). Required.
- `--tree_number` (int): An integer representing the tree number (e.g., `1`, `2`, `3`). Required.
- `--rotated` (flag): Optional. Set this flag to indicate if the setup is rotated.

### Example Usage

```bash
python3 run_pruning_bt.py --tree_type pruner --trial_number 1 --tree_number 1 --rotated
```

## System Architecture

The project consists of several core components located in their respective directories:

- **`nodes/`**: Contains individual ROS 2 nodes for data gathering, teleoperation, I/O management, and goal setting (e.g., `DataGatheringNode`, `TeleopNode`, `IOManager`).
- **`trees/`**: Defines the behavior trees, such as `rl_controller.py` for the main reinforcement learning loop and `io_tree.py` for processing inputs and outputs.
- **`behaviors/`**: Implements individual actions and condition checks used within the behavior trees (e.g., `check_termination.py`, `compute_rl_action.py`, `publish_velocity.py`).
- **`utils/`**: Utility scripts for shared operations and processing.
