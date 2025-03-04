#!/bin/bash

tmux new-session -d -s "orca_sim"

tmux split-window -h -t "orca_sim:0"

tmux send-keys -t "orca_sim:0.0" "ros2 launch stonefish_sim simulation.launch.py" C-m
tmux send-keys -t "orca_sim:0.1" "ros2 launch stonefish_sim orca_sim.launch.py" C-m

tmux attach-session -t "orca_sim"
