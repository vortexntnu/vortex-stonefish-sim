# Vortex Stonefish Sim
This repository contains scenario files and models for simulation of the Vortex NTNU drones.

## Prerequisites
- The [Stonefish](https://github.com/patrykcieslak/stonefish) library needs to be installed.
- The [Stonefish ROS 2 package](https://github.com/patrykcieslak/stonefish_ros2) must be compiled.

## Usage
Clone this repository and build your workspace.

Launch a scenario:

`ros2 launch vortex_stonefish_sim simulation.launch.py scenario_desc:=structure.scn`
