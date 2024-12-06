import py_trees
import torch as th
from pruning_sb3.algo.PPOLSTMAE import RecurrentPPOAE

class ComputeRLAction(py_trees.behaviours.Behaviour):
    """Compute an action using RL logic."""
    def __init__(self, name, model_path):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.model = self.init_rl_model(model_path)

    def init_rl_model(self, load_path):
        assert load_path is not None, "Model path not provided."
        custom_objects = {"n_envs": 1}
        self.model = RecurrentPPOAE.load(load_path, custom_objects=custom_objects)
        self.model.to(self.device)

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        if "observation" not in blackboard:
            return py_trees.common.Status.FAILURE

        observation = blackboard.observation
        # Placeholder: Replace with actual RL model inference logic
        action, self.lstm_states = self.model.predict(
            observations,  # type: ignore[arg-type]
            state=self.lstm_states,
            episode_start=True,
            deterministic=True,
        )
        blackboard.action = action
        self.logger.info(f"Computed action: {action}")
        return py_trees.common.Status.SUCCESS
