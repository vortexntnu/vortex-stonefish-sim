#!/bin/bash
 
SESSION="stonefish_sim"
WS="$HOME/ros2_ws"

 
# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null
 
# --- SIMULATION ---
tmux new-session -d -s $SESSION -n sim
 
#tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch stonefish_sim vortex_sim_launch.py scenario:=docking rendering_quality:=low scenario_config_override:=$HOME/ros2_ws/src/vortex-cv/mission/tacc/subsea_docking/subsea_docking_setup/config/sim_config.yaml" C-m
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch stonefish_sim vortex_sim_launch.py scenario:=tacc rendering_quality:=low scenario_config_override:=$HOME/ros2_ws/src/vortex-cv/mission/tacc/visual_inspection/visual_inspection_setup/config/sim_config.yaml" C-m
 
# --- CONTROL ---
tmux new-window -t $SESSION -n control
 
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch auv_setup dp_quat.launch.py" C-m
 
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch keyboard_joy keyboard_joy_node.launch.py" C-m
 
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch waypoint_manager waypoint_manager.launch.py" C-m
 
# --- PERCEPTION ---
tmux new-window -t $SESSION -n perception
 
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch visual_inspection_setup visual_inspection_fsm.launch.py" C-m
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch perception_setup ultralytics_valve_detection.launch.py" C-m
 
# --- TOOLS ---
tmux new-window -t $SESSION -n tools
 
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m
 
 
 
# --- MISSION ---
tmux new-window -t $SESSION -n mission
 
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch landmark_server landmark_server.launch.py" C-m
 
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 run yasmin_viewer yasmin_viewer_node" C-m
 
# --- Pipeline detectio ---
tmux new-window -t $SESSION -n pipe_det

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch vortex_pipeline_image_endpoints image_endpoints.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch vortex_image_segmentation yolo_segmentation.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch vortex_position_estimator position_estimator.launch.py" C-m


# --- MISSION ---
tmux new-window -t $SESSION -n gripper
#tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch gripper_open_loop gripper_open_loop.launch.py" C-m
 
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch gripper_reference_filter gripper_reference_filter.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch gripper_controller gripper_controller.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && export ROS_DOMAIN_ID=42 && source install/setup.bash && ros2 launch gripper_sim_interface gripper_sim_interface.launch.py" C-m
 
 
# Nice layout
tmux select-layout tiled
 
# Attach to session
tmux attach-session -t $SESSION