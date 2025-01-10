import py_trees_ros
import py_trees
from py_trees_ros.trees import BehaviourTree
from pruning_bt.behaviors.process_io import ProcessIO

def create_io_processing_tree(asyncio_loop)-> BehaviourTree:
    """Construct the behavior tree."""
    root = py_trees.composites.Sequence("Root", memory=False)
    process_io = ProcessIO(name="Process IO", asyncio_loop=asyncio_loop)
    root.add_children([process_io])
    return root

      
