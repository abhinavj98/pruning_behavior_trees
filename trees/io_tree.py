import py_trees_ros
import py_trees
from py_trees_ros.trees import BehaviourTree
from pruning_bt.behaviors.process_io import ProcessIO

def create_io_processing_tree(asyncio_loop, tree_type, tree_number, rotated)-> BehaviourTree:
    """Construct the behavior tree."""
    root = py_trees.composites.Sequence("Root", memory=False)
    process_io = ProcessIO(name="Process IO", asyncio_loop=asyncio_loop, tree_type=tree_type, tree_number = tree_number, rotated=rotated)
    root.add_children([process_io])
    return root

      
