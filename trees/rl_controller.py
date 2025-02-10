import py_trees
from pruning_bt.behaviors.aggregate_observation import AggregateObservation
from pruning_bt.behaviors.rviz_viz import RVizVisualization
from pruning_bt.behaviors.compute_rl_action import ComputeRLAction
from pruning_bt.behaviors.check_termination import CheckTermination
import numpy as np
import sys
# from behaviors.compute_rl_action import ComputeRLAction
# from behaviors.publish_velocity import PublishVelocity
# from behaviors.rviz_viz import RVizMarkerPublisher
class MockAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.counter = 0    
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info(f"Executing {self.name}, counter={self.counter}")
        self.blackboard.set("action", np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3]))
        self.counter += 1
        if self.counter < 3:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
        
def create_rl_controller_tree(asyncio_loop):
    """Construct the behavior tree."""
    #Export location of pruning_sb3 to PYTHONPATH
    
    
    root = py_trees.composites.Sequence("Root", memory=True)
    rl_model_path = "/home/grimmlins/bt_pruning_ws/src/pruning_bt/weights"
    load_folder = "field_trials_ufo_bc"#"field_trials"
    load_timestep = 1920000#1428000
    #  Aggregate Observation
    aggregate_observation = AggregateObservation(name="Aggregate Observation")
    rviz_vizualization = RVizVisualization(name="RViz Marker Publisher", asyncio_loop = asyncio_loop)
    compute_rl_action = ComputeRLAction(name="Compute RL Action", model_path=rl_model_path, 
                                        load_timestep=load_timestep, load_folder = load_folder, 
                                        asyncio_loop=asyncio_loop)
    check_termination = CheckTermination(name = "Check Termination")
    # Compute Action
    # compute_action = ComputeRLAction(name="Compute Action", model_path=model_path)
    # # Publish Velocity
    # publish_velocity = PublishVelocity(name="Publish Velocity", publisher=publisher)
    # # RViz Marker Publisher
    # rviz_marker_publisher = RVizMarkerPublisher(name="RViz Marker Publisher", node=publisher.node)

    # Add to the tree
    root.add_children([aggregate_observation, compute_rl_action, check_termination, rviz_vizualization])#, compute_action, publish_velocity, rviz_marker_publisher])
    return root
