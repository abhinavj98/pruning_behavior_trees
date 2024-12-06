import py_trees
from behaviors.aggregate_observation import AggregateObservation
from behaviors.compute_rl_action import ComputeRLAction
from behaviors.publish_velocity import PublishVelocity
from behaviors.rviz_viz import RVizMarkerPublisher

def rl_controller_tree(publisher, model_path):
    """Construct the behavior tree."""
    root = py_trees.composites.Sequence("Root")

    #  Aggregate Observation
    aggregate_observation = AggregateObservation(name="Aggregate Observation")
    # Compute Action
    compute_action = ComputeRLAction(name="Compute Action", model_path=model_path)
    # Publish Velocity
    publish_velocity = PublishVelocity(name="Publish Velocity", publisher=publisher)
    # RViz Marker Publisher
    rviz_marker_publisher = RVizMarkerPublisher(name="RViz Marker Publisher", node=publisher.node)

    # Add to the tree
    root.add_children([aggregate_observation, compute_action, publish_velocity, rviz_marker_publisher])
    return root
