# mujoco_mpc_deploy

This repository contains the code for deploying MuJoCo MPC on Unitree Robots. (This is still a work in progress)

## Installation

1. install the python interface for MuJoCo MPC following the [official instructions](https://github.com/google-deepmind/mujoco_mpc)
2. install necessary [Unitree SDK](https://github.com/unitreerobotics/unitree_legged_sdk) or [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) dependecies
3. the estimation module is still under development. for now, we use optitrack and communciate using ROS. in principle, you can use any mocap or onboard estimation module.
4. install the interface ```pip install -e .```
5. 