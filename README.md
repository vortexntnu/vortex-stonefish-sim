# Vortex Stonefish Sim
This repository contains scenario files and models for simulation of the Vortex NTNU drones.

## Prerequisites
- The [Stonefish](https://github.com/patrykcieslak/stonefish) library needs to be installed.
- The [Stonefish ROS 2 package](https://github.com/patrykcieslak/stonefish_ros2) must be compiled.

## Usage
Clone this repository and build your workspace.

Launch a scenario:

```bash
ros2 launch stonefish_sim simulation.launch.py task:=structure
```

![Image](https://drive.google.com/uc?export=view&id=1Mdg5cXCWC3h63GH70mxJqeO6YI-iAcV7)

![Image](https://drive.google.com/uc?export=view&id=1elYr7ipYRVbe5FjxG1w3hYaTKhNYyyDH)


If you have a less capable computer, other options are to use low rendering quality or turn off rendering completely. For low rendering quality use

```bash
ros2 launch stonefish_sim simulation.launch.py rendering_quality:=low
```

and for no rendering use

```bash
ros2 launch stonefish_sim simulation_nogpu.launch.py
```
