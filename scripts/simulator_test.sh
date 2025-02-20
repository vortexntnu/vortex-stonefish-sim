#!/bin/bash
set -e

setsid ros2 launch stonefish_sim simulation_nogpu.launch.py &
SIM_PID=$!
echo "Launced simulator with PID: $SIM_PID"

echo "Waiting for simulator to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/orca/odom"; do sleep 1; done'
echo "Simulator started"

echo "Waiting for odom data..."
timeout 10s ros2 topic echo /orca/odom --once
echo "Got odom data"

setsid ros2 launch stonefish_sim orca_sim.launch.py &
ORCA_PID=$!
echo "Launced orca with PID: $ORCA_PID"

echo "Waiting for sim interface to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/orca/pose"; do sleep 1; done'
echo "Simulator started"

echo "Waiting for odom data..."
timeout 10s ros2 topic echo /orca/pose --once
echo "Got pose data"

setsid ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py &
CONTROLLER_PID=$!
echo "Launced controller with PID: $CONTROLLER_PID"

setsid ros2 launch reference_filter_dp reference_filter.launch.py &
FILTER_PID=$!
echo "Launced filter with PID: $FILTER_PID"

echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 topic pub /orca/killswitch std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /orca/operation_mode std_msgs/msg/String "{data: 'autonomous mode'}" -1

echo "Sending goal"
python3 scripts/send_goal.py

kill -TERM -"$SIM_PID" -"$ORCA_PID" -"$CONTROLLER_PID" -"$FILTER_PID"