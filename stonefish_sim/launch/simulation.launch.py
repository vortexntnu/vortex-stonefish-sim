import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node

gpu_scenarios = [
    "default",
    "docking",
    "pipeline",
    "structure",
    "orca_demo",
    "freya_demo",
    "orca_freya_demo",
    "tacc",
]
no_gpu_scenarios = [
    "orca_no_gpu",
    "freya_no_gpu",
]


def load_scenario_config(scenario_val):
    config_path = os.path.join(
        get_package_share_directory("stonefish_sim"),
        "config",
        f"{scenario_val}_config.yaml",
    )

    if not os.path.exists(config_path):
        print(
            f"Warning: Scenario config not found for scenario '{scenario_val}'. Using default config."
        )
        config_path = os.path.join(
            get_package_share_directory("stonefish_sim"),
            "config",
            "default_config.yaml",
        )

    with open(config_path) as f:
        config = yaml.safe_load(f)

    return config


def get_sim_node(context, scenario_config=None):
    scenario_val = LaunchConfiguration("scenario").perform(context)
    rendering_enabled = (
        LaunchConfiguration("rendering").perform(context).lower() == "true"
    )

    if scenario_config is None:
        scenario_config = load_scenario_config(scenario_val)

    sim_data = LaunchConfiguration("simulation_data")
    sim_rate = LaunchConfiguration("simulation_rate")
    win_x = LaunchConfiguration("window_res_x")
    win_y = LaunchConfiguration("window_res_y")
    rend_qual = LaunchConfiguration("rendering_quality")

    stonefish_dir = get_package_share_directory("stonefish_sim")
    scenario_file = PathJoinSubstitution(
        [stonefish_dir, "scenarios", TextSubstitution(text=f"{scenario_val}.scn")]
    )

    if rendering_enabled:
        exec_name = "stonefish_simulator"
        args = [sim_data, scenario_file, sim_rate, win_x, win_y, rend_qual]
    elif not rendering_enabled and scenario_val not in no_gpu_scenarios:
        # Default to orca_no_gpu.scn
        scenario_file = PathJoinSubstitution(
            [stonefish_dir, "scenarios", TextSubstitution(text="orca_no_gpu.scn")]
        )
        exec_name = "stonefish_simulator_nogpu"
        args = [sim_data, scenario_file, sim_rate]
    else:
        exec_name = "stonefish_simulator_nogpu"
        args = [sim_data, scenario_file, sim_rate]

    return Node(
        package="stonefish_ros2",
        executable=exec_name,
        namespace="stonefish_ros2",
        name=exec_name,
        arguments=args,
        parameters=[scenario_config],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    scenario_val = LaunchConfiguration("scenario").perform(context)
    override_path = LaunchConfiguration("scenario_config_override").perform(context)
    drone_val = LaunchConfiguration("drone").perform(context)

    if override_path and os.path.exists(override_path):
        with open(override_path) as f:
            scenario_config = yaml.safe_load(f)
    elif scenario_val in gpu_scenarios + no_gpu_scenarios:
        scenario_config = load_scenario_config(scenario_val)
    else:
        scenario_config = {}

    drone_config = {"drone_file": f"{drone_val}.scn"}
    merged_config = {**scenario_config, **drone_config}

    return [get_sim_node(context, scenario_config=merged_config)]


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
                "scenario",
                default_value="default",
                description=(
                    "Scenario to load. Use one of "
                    f"{gpu_scenarios + no_gpu_scenarios}, or leave as 'default' "
                    "to choose automatically."
                ),
            ),
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="drone.scn file to use",
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
                "window_res_x", default_value="1920", description="Render window width"
            ),
            DeclareLaunchArgument(
                "window_res_y", default_value="1080", description="Render window height"
            ),
            DeclareLaunchArgument(
                "rendering_quality",
                default_value="high",
                description="Rendering quality (high/med/low)",
            ),
            DeclareLaunchArgument(
                "scenario_config_override",
                default_value="",
                description="Path to override scenario config YAML",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
