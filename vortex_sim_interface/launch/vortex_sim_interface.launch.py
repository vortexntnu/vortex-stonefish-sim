import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mock_odom = LaunchConfiguration("mock_odom").perform(context).lower() == "true"
    drone, namespace = resolve_drone_and_namespace(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    drone_description_launch = os.path.join(
        get_package_share_directory("auv_setup"),
        "launch",
        "drone_description.launch.py",
    )

    remappings = []
    if not mock_odom:
        remappings = [
            (f"/{namespace}/odom", f"/{namespace}/odom/stonefish"),
            (f"/{namespace}/pose", f"/{namespace}/pose/stonefish"),
            (f"/{namespace}/twist", f"/{namespace}/twist/stonefish"),
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drone_description_launch),
            launch_arguments={
                "drone": drone,
                "namespace": namespace,
            }.items(),
        ),
        Node(
            package="vortex_sim_interface",
            executable="vortex_sim_interface",
            name="vortex_sim_interface",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                {"mock_odom": mock_odom},
                {"tf_name_prefix": namespace},
                drone_params,
            ],
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock_odom",
                default_value="true",
                description="If true, the sim will publish odometry and corresponding tf2 transform. Only set to false if using an external localization system.",
            ),
        ]
        + declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
