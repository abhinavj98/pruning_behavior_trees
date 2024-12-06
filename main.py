from rclpy.executors import MultiThreadedExecutor
import rclpy
from trees.rl_controller import rl_controller_tree
from nodes.data_gathering_node import DataGatheringNode
def main():
    rclpy.init()

    # Create Data Gathering Node
    data_gathering_node = DataGatheringNode()

    # Create Behavior Tree
    model_path = "model.pth"  # Update with your model's path
    behavior_tree = rl_controller_tree()
    behavior_tree.setup(timeout=15)

    # Multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(data_gathering_node)
    executor.add_node(behavior_tree.node)

    try:
        behavior_tree.tick_tock(period_ms=100)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
