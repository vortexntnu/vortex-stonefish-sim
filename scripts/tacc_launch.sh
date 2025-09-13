#!/bin/bash

# Navigate to the ROS 2 workspace directory
cd /home/vortex/stonefish_ws

# Source the ROS 2 setup file to set up the environment
source install/setup.bash

# Launch the Stonefish simulation in the background
ros2 launch stonefish_sim simulation.launch.py task:=tacc rendering_quality:=low &

# Launch the Orca simulation
ros2 launch stonefish_sim orca_sim.launch.py &

# Launch the Foxglove Bridge in the background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

# Launch Foxglove Studio
foxglove-studio &

# Wait for all background processes to finish
wait