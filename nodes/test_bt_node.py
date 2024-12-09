import rclpy
from rclpy.node import Node
import py_trees


class TestBehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('test_behavior_tree_node')
        self.get_logger().info('Starting Test Behavior Tree Node...')

        # Create a behavior tree
        self.behavior_tree = self.create_behavior_tree()

        # Create a timer to tick the behavior tree every 100 ms
        self.create_timer(0.1, self.tick_tree)

    def create_behavior_tree(self):
        # Create the root of the behavior tree
        root = py_trees.composites.Sequence("Root", memory=False)

        # Add two simple tasks
        task_hello = PrintHello()
        task_world = PrintWorld()

        root.add_children([task_hello, task_world])
        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        # Tick the behavior tree
        print(py_trees.display.ascii_tree(self.behavior_tree.root))
        self.behavior_tree.tick()
        print(py_trees.display.ascii_tree(self.behavior_tree.root))

# Simple task: Print "Hello"
class PrintHello(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHello"):
        super().__init__(name)

    def update(self):
        print("Hello")
        return py_trees.common.Status.SUCCESS


# Simple task: Print "World"
class PrintWorld(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintWorld"):
        super().__init__(name)

    def update(self):
        print("World")
        return py_trees.common.Status.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = TestBehaviorTreeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
