#!/bin/bash

tmux new-session -d -s "freya_sim"

tmux split-window -h -t "freya_sim:0"

tmux send-keys -t "freya_sim:0.0" "ros2 launch stonefish_sim simulation.launch.py task:=freya_demo" C-m
tmux send-keys -t "freya_sim:0.1" "ros2 launch stonefish_sim freya_sim.launch.py" C-m

tmux attach-session -t "freya_sim"
