import mujoco
from mujoco_mpc import agent as agent_lib
import pathlib
import numpy as np

# Get the absolute path to the mujoco_mpc module
mujoco_mpc_path = pathlib.Path(agent_lib.__file__).resolve().parent

# quadruped model
model_path = (
    mujoco_mpc_path / "mjpc/tasks/quadruped/task_flat.xml"
)
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)


with agent_lib.Agent(
    server_binary_path=pathlib.Path(agent_lib.__file__).parent
    / "mjpc"
    / "ui_agent_server",
    task_id="Quadruped Flat",
    model=model,
) as agent:
  while True:

    pass


