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
import os
import yaml

gpu_tasks = [
    "default",
    "docking",
    "pipeline",
    "structure",
    "orca_demo",
    "freya_demo",
    "orca_freya_demo",
]
no_gpu_tasks = [
    "orca_no_gpu",
    "freya_no_gpu",
]


def validate_task(task_val, rendering_enabled):
    if rendering_enabled and task_val not in gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires rendering to be disabled. Valid GPU tasks: {gpu_tasks}"
        )
    if not rendering_enabled and task_val not in no_gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires rendering to be enabled. Valid no-GPU tasks: {no_gpu_tasks}"
        )


def load_scenario_config(task_val):
    config_path = os.path.join(
        get_package_share_directory("stonefish_sim"), "config", f"{task_val}_config.yaml"
    )

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Missing scenario config: {config_path}")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    return config


def get_sim_node(context, scenario_config=None):
    task_val = LaunchConfiguration("task").perform(context)
    rendering_enabled = LaunchConfiguration("rendering").perform(context).lower() == "true"

    validate_task(task_val, rendering_enabled)

    if scenario_config is None:
        scenario_config = load_scenario_config(task_val)

    sim_data = LaunchConfiguration("simulation_data")
    sim_rate = LaunchConfiguration("simulation_rate")
    win_x = LaunchConfiguration("window_res_x")
    win_y = LaunchConfiguration("window_res_y")
    rend_qual = LaunchConfiguration("rendering_quality")

    stonefish_dir = get_package_share_directory("stonefish_sim")
    scenario_file = PathJoinSubstitution([
        stonefish_dir, "scenarios", TextSubstitution(text=f"{task_val}.scn")
    ])

    if rendering_enabled:
        exec_name = "stonefish_simulator"
        args = [sim_data, scenario_file, sim_rate, win_x, win_y, rend_qual]
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
        output="screen"
    )


def launch_setup(context, *args, **kwargs):
    task_val = LaunchConfiguration("task").perform(context)
    override_path = LaunchConfiguration("scenario_config_override").perform(context)

    if override_path and os.path.exists(override_path):
        with open(override_path, 'r') as f:
            scenario_config = yaml.safe_load(f)
    else:
        scenario_config = None

    return [get_sim_node(context, scenario_config=scenario_config)]


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
            description="Path to override scenario config YAML"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
