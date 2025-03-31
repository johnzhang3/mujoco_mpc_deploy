# mujoco_mpc_deploy

This repository contains the code for deploying MuJoCo MPC on Unitree Robots. (This is still a work in progress). The goal for this repository is to provide a hardware interface. The MPC solver and tasks are hosted on the official [MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc) repository. Robot models are hosted on the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). If you'd like to add or modify tasks or robot mdoels please submit PRs to those repos directly.

For result videos and details, please checkout out [paper](https://arxiv.org/pdf/2503.04613) and [website](https://johnzhang3.github.io/mujoco_ilqr/).

## Installation

1. install the python interface for MuJoCo MPC following the [official instructions](https://github.com/google-deepmind/mujoco_mpc)
2. install necessary [Unitree SDK](https://github.com/unitreerobotics/unitree_legged_sdk) or [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) dependecies
3. the estimation module is still under development. for now, we use optitrack and communciate using ROS. in principle, you can use any mocap or onboard estimation module.
4. install the interface ```pip install -e .```

## Examples

1. to verify things are working correctly in simulation, run: ```python examples/mjpc_gui.py```

## Citing
If you find this work helpful, please consider citing our paper:
```
@misc{zhang2025wholebodymodelpredictivecontrollegged,
        title={Whole-Body Model-Predictive Control of Legged Robots with MuJoCo}, 
        author={John Z. Zhang and Taylor A. Howell and Zeji Yi and Chaoyi Pan and Guanya Shi and Guannan Qu and Tom Erez and Yuval Tassa and Zachary Manchester},
        year={2025},
        eprint={2503.04613},
        archivePrefix={arXiv},
        primaryClass={cs.RO},
        url={https://arxiv.org/abs/2503.04613}, 
  }
```