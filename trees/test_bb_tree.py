import py_trees
from py_trees.blackboard import Blackboard

class PrintHelloMessage(py_trees.behaviour.Behaviour):
    """
    A behavior that reads the blackboard and prints the `hello_message`.
    """

    def __init__(self, name="PrintHelloMessage"):
        super().__init__(name)
        self.blackboard = Blackboard()

    def update(self):
        # Check if the message exists
        try:
            self.blackboard.get("hello_message")
        except KeyError:
            self.logger.warning("No message on the blackboard!")
            return py_trees.common.Status.FAILURE
        # Print the message
        self.logger.info(f"Blackboard Message: {self.blackboard.get('hello_message')}")
        return py_trees.common.Status.SUCCESS


def create_tree():
    """
    Creates a simple behavior tree that prints the blackboard message.
    """
    root = py_trees.composites.Sequence("Root", memory=True)
    print_message = PrintHelloMessage(name="Print Blackboard Message")
    root.add_children([print_message])
    return root
