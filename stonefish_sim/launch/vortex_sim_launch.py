import os
from os import path

import yaml
from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    "nautilus_no_gpu",
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


def get_sim_node(
    context: LaunchContext,
    drone: str,
    namespace: str,
    scenario_config=None,
):
    scenario_val = LaunchConfiguration("scenario").perform(context)
    rendering_enabled = (
        LaunchConfiguration("rendering").perform(context).lower() == "true"
    )
    mock_odom = LaunchConfiguration("mock_odom").perform(context).lower() == "true"

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
    elif scenario_val not in no_gpu_scenarios:
        scenario_file = PathJoinSubstitution(
            [stonefish_dir, "scenarios", TextSubstitution(text="orca_no_gpu.scn")]
        )
        exec_name = "stonefish_simulator_nogpu"
        args = [sim_data, scenario_file, sim_rate]
    else:
        exec_name = "stonefish_simulator_nogpu"
        args = [sim_data, scenario_file, sim_rate]

    remappings = [
        ("/stonefish/thrusters", f"/{namespace}/stonefish/thrusters"),
    ]

    if not mock_odom:
        remappings.append((f"/{namespace}/odom", f"/{namespace}/odom/stonefish"))

    return Node(
        package="stonefish_ros2",
        executable=exec_name,
        namespace="stonefish_ros2",
        name=exec_name,
        arguments=args,
        parameters=[scenario_config],
        remappings=remappings,
        output="screen",
    )


def launch_setup(context: LaunchContext, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    mock_odom = LaunchConfiguration("mock_odom").perform(context)
    scenario_val = LaunchConfiguration("scenario").perform(context)
    override_path = LaunchConfiguration("scenario_config_override").perform(context)
    solver_type = LaunchConfiguration("solver_type").perform(context)

    common_launch_args = {
        "drone": drone,
        "namespace": namespace,
    }.items()

    thrust_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("thrust_allocator_auv"),
                "launch",
                "thrust_allocator_auv.launch.py",
            )
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
            "solver_type": solver_type,
        }.items(),
    )

    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("joystick_interface_auv"),
                "launch",
                "joystick_interface_auv.launch.py",
            )
        ),
        launch_arguments=common_launch_args,
    )

    operation_mode_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("operation_mode_manager"),
                "launch",
                "operation_mode_manager.launch.py",
            )
        ),
        launch_arguments=common_launch_args,
    )

    stonefish_sim_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("stonefish_sim_interface"),
                "launch",
                "stonefish_sim_interface.launch.py",
            )
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
            "mock_odom": mock_odom,
        }.items(),
    )

    if override_path and os.path.exists(override_path):
        with open(override_path) as f:
            scenario_config = yaml.safe_load(f)
    elif scenario_val in gpu_scenarios + no_gpu_scenarios:
        scenario_config = load_scenario_config(scenario_val)
    else:
        scenario_config = {}

    if (
        "drone_position" in scenario_config
        and f"{drone}_position" not in scenario_config
    ):
        scenario_config[f"{drone}_position"] = scenario_config["drone_position"]

    if (
        "drone_orientation" in scenario_config
        and f"{drone}_orientation" not in scenario_config
    ):
        scenario_config[f"{drone}_orientation"] = scenario_config["drone_orientation"]

    merged_config = {
        **scenario_config,
        "drone_file": f"{drone}.scn",
    }

    sim_node = get_sim_node(
        context=context,
        drone=drone,
        namespace=namespace,
        scenario_config=merged_config,
    )

    keyboard_joy_enabled = (
        LaunchConfiguration("keyboard_joy").perform(context).lower() == "true"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver",
        output="screen",
        parameters=[
            {
                "deadzone": 0.15,
                "autorepeat_rate": 100.0,
            }
        ],
        remappings=[
            ("/joy", f"/{namespace}/joy"),
        ],
    )

    nodes = [
        thrust_allocator_launch,
        joystick_interface_launch,
        operation_mode_manager_launch,
        stonefish_sim_interface_launch,
        sim_node,
        joy_node,
    ]

    if keyboard_joy_enabled:
        keyboard_joy_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                path.join(
                    get_package_share_directory("keyboard_joy"),
                    "launch",
                    "keyboard_joy_node.launch.py",
                )
            ),
            launch_arguments=common_launch_args,
        )
        nodes.append(keyboard_joy_launch)

    return nodes


def generate_launch_description():
    stonefish_dir = get_package_share_directory("stonefish_sim")

    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT",
        value="[${severity}] [${time}] [${node}]: ${message}",
    )

    set_warn_color = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT",
        value="1",
    )

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
                "mock_odom",
                default_value="true",
                description="If true, the sim will publish odometry and corresponding tf2 transform. Only set to false if using an external localization system.",
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
                "window_res_x",
                default_value="1920",
                description="Render window width",
            ),
            DeclareLaunchArgument(
                "window_res_y",
                default_value="1080",
                description="Render window height",
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
            DeclareLaunchArgument(
                "keyboard_joy",
                default_value="false",
                description="Launch keyboard joy node (true/false)",
            ),
            DeclareLaunchArgument(
                "solver_type",
                default_value="qp",
                description="Thrust allocator solver type (available: pseudoinverse, qp)",
            ),
        ]
        + declare_drone_and_namespace_args()
        + [
            set_env_var,
            set_warn_color,
            OpaqueFunction(function=launch_setup),
        ]
    )
