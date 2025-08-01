"""Testing Template for Stonefish Simulation.

This file serves as a template for setting up tests in Stonefish with
varying starting parameters.

1. Add your custom scenario to the appropriate task list.
2. Define any custom parameters you need for the scenario.
3. Add them to the node parameters section and make sure the names match the parameters in the scenario file(s).
   In scenario file: "$(param <your_parameter_name>)". Look at the defaults for examples.
"""

import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node

vortex_stonefish_sim_dir = get_package_share_directory("stonefish_sim")
simulation_data_default = PathJoinSubstitution([vortex_stonefish_sim_dir, "data"])

### 1. Add your testing scenario here ###
gpu_tasks = [
    "default_gpu",
]
no_gpu_tasks = [
    "default_no_gpu",
]
#########################################


def generate_random_position():
    """Example function to generate a random starting position."""
    x = random.uniform(-10.0, 10.0)
    y = random.uniform(-10.0, 10.0)
    z = random.uniform(0.0, 2.0)
    return f"{x} {y} {z}"


### 2. Your custom parameters ###
drone_starting_position = generate_random_position()
drone_starting_orientation = "0.0 0.0 0.0"

#################################


class ConcatenateSubstitutions(Substitution):
    def __init__(self, *substitutions):
        self.substitutions = substitutions

    def perform(self, context):
        return "".join([sub.perform(context) for sub in self.substitutions])


def launch_setup(context, *args, **kwargs):
    rendering_enabled = (
        LaunchConfiguration("rendering").perform(context).lower() == "true"
    )

    task_arg = LaunchConfiguration("task").perform(context)
    if task_arg == "auto":
        task_val = "default_gpu" if rendering_enabled else "default_no_gpu"
    else:
        task_val = task_arg

    if rendering_enabled and task_val not in gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires GPU rendering to be disabled. "
            f"Choose one of: {', '.join(gpu_tasks)}"
        )
    if not rendering_enabled and task_val not in no_gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires GPU rendering to be enabled. "
            f"Choose one of: {', '.join(no_gpu_tasks)}"
        )

    sim_data = LaunchConfiguration("simulation_data")
    sim_rate = LaunchConfiguration("simulation_rate")
    win_x = LaunchConfiguration("window_res_x")
    win_y = LaunchConfiguration("window_res_y")
    rend_qual = LaunchConfiguration("rendering_quality")

    stonefish_dir = get_package_share_directory("stonefish_sim")
    scenario = PathJoinSubstitution(
        [stonefish_dir, "scenarios/tests", TextSubstitution(text=f"{task_val}.scn")]
    )

    if rendering_enabled:
        exe = "stonefish_simulator"
        node_args = [sim_data, scenario, sim_rate, win_x, win_y, rend_qual]
        node_name = "stonefish_simulator"
    else:
        exe = "stonefish_simulator_nogpu"
        node_args = [sim_data, scenario, sim_rate]
        node_name = "stonefish_simulator_nogpu"

    node = Node(
        package="stonefish_ros2",
        executable=exe,
        namespace="stonefish_ros2",
        name=node_name,
        arguments=node_args,
        ### 3. Add parameters to the node ###
        ### NB: Name (the string) must match the parameter name in the .scn file ###
        parameters=[
            {
                "position": drone_starting_position,
                "orientation": drone_starting_orientation,
            }
        ],
        ############################################################################
        output="screen",
    )

    return [node]


def generate_launch_description():
    stonefish_dir = get_package_share_directory("stonefish_sim")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rendering",
                default_value="true",
                description="Enable GPU rendering (true/false)",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="auto",
                description=(
                    "Scenario to load. Use one of "
                    f"{gpu_tasks + no_gpu_tasks}, or leave as 'auto' "
                    "to choose automatically."
                ),
            ),
            DeclareLaunchArgument(
                "simulation_data",
                default_value=PathJoinSubstitution([stonefish_dir, "data"]),
                description="Path to the simulation data folder",
            ),
            DeclareLaunchArgument(
                "simulation_rate",
                default_value="100.0",
                description="Physics update rate [Hz]",
            ),
            DeclareLaunchArgument(
                "window_res_x", default_value="2460", description="Render window width"
            ),
            DeclareLaunchArgument(
                "window_res_y", default_value="1340", description="Render window height"
            ),
            DeclareLaunchArgument(
                "rendering_quality",
                default_value="high",
                description="Rendering quality (high/med/low)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
