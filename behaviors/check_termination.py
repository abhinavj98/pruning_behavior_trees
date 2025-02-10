import py_trees
from vl6180_msgs.msg import Vl6180
import numpy as np
class CheckTermination(py_trees.behaviour.Behaviour):
    """Terminate the execution if tof sensor gets a certain reading or 
     steps get exceeded """
    
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.max_steps = 80
        self.tof_data_topic = "/microROS/vl6180/data"
        self.tof_data = np.array([255, 255])

    def setup(self, node):
        self.blackboard.set("is_terminated", False)
        self.node = node
        self.tof_sub = self.node.create_subscription(Vl6180, self.tof_data_topic,
                                                     self._handle_tof_data, 1)
        
    def _handle_tof_data(self, msg):
        self.tof_data = np.array(msg.data)

    def update(self):
        if self.max_steps > 150:
            self.node.get_logger().info("Reached max steps. Stopping")
            self.blackboard.set("is_terminated", True)

        distance = np.linalg.norm(self.blackboard.get("observation")['relative_distance'])

        if distance < 0.05 and np.any(self.tof_data < 150):
            self.node.get_logger().info("Got sensor reading. Stopping")
            self.node.get_logger().info(f"Distance {distance}")
            
            # self.blackboard.set("is_terminated", True)
            self.node.get_logger().info(f"{self.tof_data}")
        return py_trees.common.Status.SUCCESS

    
