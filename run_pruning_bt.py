import rclpy
from rclpy.executors import MultiThreadedExecutor
from pruning_bt.nodes.data_gathering_node import DataGatheringNode
from pruning_bt.nodes.set_goal_service import SetGoalService
from pruning_bt.nodes.teleop_node import TeleopNode
from pruning_bt.nodes.io_manager import IOManager
from pruning_bt.trees.rl_controller import create_rl_controller_tree
from pruning_bt.trees.io_tree import create_io_processing_tree
import py_trees_ros
import asyncio
import threading
import sys
import argparse
import os
# sys.path.append("/home/grimmlins/bt_pruning_ws/src/pruning_bt/pruning_sb3/")
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    # Set up argparse to get tree_type, trial_number, and tree_number from command line arguments
    parser = argparse.ArgumentParser(description="Start the pruning behavior tree.")
    parser.add_argument('--tree_type', type=str, required=True, help="Type of the tree (e.g., 'pruner', 'test').")
    parser.add_argument('--trial_number', type=int, required=True, help="Trial number (e.g., 1, 2, 3).")
    parser.add_argument('--tree_number', type=int, required=True, help="Tree number (e.g., 1, 2, 3).")
    parser.add_argument('--rotated', action='store_true', help='Set flag to True')
    
    # Parse the arguments
    args = parser.parse_args()

    # Extract the parameters
    tree_type = args.tree_type
    trial_number = args.trial_number
    tree_number = args.tree_number
    rotated = args.rotated

    sys.path.append("/home/grimmlins/bt_pruning_ws/src/pruning_bt/")
    rclpy.init()

    # Create Data Gathering Node
    data_gathering_node = DataGatheringNode()

    # Pass the parameters to SetGoalService
    set_goal_service = SetGoalService(tree_type=tree_type, tree_number=tree_number, trial_number=trial_number)

    io_manager = IOManager()
    teleop_node = TeleopNode()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop_thread = threading.Thread(target=loop.run_forever, daemon=True)
    loop_thread.start()

    behavior_tree = py_trees_ros.trees.BehaviourTree(create_rl_controller_tree(asyncio_loop=loop))
    io_behavior_tree = py_trees_ros.trees.BehaviourTree(create_io_processing_tree(asyncio_loop = loop, tree_type=tree_type, tree_number=tree_number, rotated=rotated))

    # Create a parallel tree to run both trees
    behavior_tree.setup(timeout=400)
    io_behavior_tree.setup(timeout=15)

    data_gathering_node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
    set_goal_service.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
    io_manager.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

    # Multi-threaded executor to run the nodes
    executor = MultiThreadedExecutor()
    executor.add_node(data_gathering_node)
    executor.add_node(behavior_tree.node)
    executor.add_node(set_goal_service)
    executor.add_node(io_manager)
    executor.add_node(io_behavior_tree.node)
    executor.add_node(teleop_node)

    try:
        # Execute the behavior trees
        behavior_tree.tick_tock(period_ms=200)
        io_behavior_tree.tick_tock(period_ms=200)
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Shutting down...")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
