from rclpy.executors import MultiThreadedExecutor
import rclpy
# from pruning_bt.trees.rl_controller import rl_controller_tree
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

# sys.path.append("/home/grimmlins/bt_pruning_ws/src/pruning_bt/pruning_sb3/")
def main():
    sys.path.append("/home/grimmlins/bt_pruning_ws/src/pruning_bt/")
    rclpy.init()
    #Set Logging Level to Fatal

    # Create Data Gathering Node
    data_gathering_node = DataGatheringNode()
    set_goal_service = SetGoalService()
    io_manager = IOManager()
    teleop_node = TeleopNode()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop_thread = threading.Thread(target=loop.run_forever, daemon=True)
    loop_thread.start()
    behavior_tree = py_trees_ros.trees.BehaviourTree(create_rl_controller_tree(asyncio_loop=loop))
    io_behavior_tree = py_trees_ros.trees.BehaviourTree(create_io_processing_tree(asyncio_loop = loop))
    #Create a parallel tree to run both trees
    behavior_tree.setup(timeout=400)
    io_behavior_tree.setup(timeout=15)

    data_gathering_node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
    set_goal_service.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
    io_manager.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
    # behavior_tree.node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

    # Create Behavior Tree
    # model_path = "model.pth"  # Update with your model's path
    # behavior_tree = rl_controller_tree()
    behavior_tree.setup(timeout=15)

    # Multi-threaded executor
    #Launch everything from the same process so that blackboard is shared
    executor = MultiThreadedExecutor()
    executor.add_node(data_gathering_node)
    executor.add_node(behavior_tree.node)
    executor.add_node(set_goal_service)
    executor.add_node(io_manager)
    executor.add_node(io_behavior_tree.node)
    executor.add_node(teleop_node)
    try:
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
