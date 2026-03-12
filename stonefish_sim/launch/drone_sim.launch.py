from os import path

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

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
        launch_arguments=common_launch_args,
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
        launch_arguments=common_launch_args,
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

    return [
        thrust_allocator_launch,
        joystick_interface_launch,
        operation_mode_manager_launch,
        stonefish_sim_interface_launch,
        joy_node,
    ]


def generate_launch_description():
    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT",
        value="[${severity}] [${time}] [${node}]: ${message}",
    )

    set_warn_color = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT",
        value="1",
    )

    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            set_env_var,
            set_warn_color,
            OpaqueFunction(function=launch_setup),
        ]
    )
