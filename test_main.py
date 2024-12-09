from rclpy.executors import MultiThreadedExecutor
import rclpy
from pruning_bt.nodes.test_bb_node import BlackboardUpdater
from pruning_bt.trees.test_bb_tree import create_tree
import py_trees_ros

def main():
    rclpy.init()

    # Create ROS2 Node to update the blackboard
    updater_node = BlackboardUpdater()

    # Create the behavior tree
    behavior_tree = py_trees_ros.trees.BehaviourTree(create_tree())
    behavior_tree.setup(timeout=15)

    # Multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(updater_node)
    executor.add_node(behavior_tree.node)

    try:
        print("Running...")
        behavior_tree.tick_tock(period_ms=100)  # Tick the behavior tree every 100ms
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
