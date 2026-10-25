from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        "num_drones", default_value="1", description="Number of drones to control"
    )

    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT", value="[${severity}] [${time}] [${node}]: ${message}"
    )

    set_warn_color = SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1")

    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("joystick_interface_asv"),
                "launch",
                "joystick_interface_asv.launch.py",
            )
        )
    )

    thrust_allocator_asv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("thrust_allocator_asv"),
                "launch",
                "thrust_allocator_asv.launch.py",
            )
        )
    )

    def include_joy_node(context: LaunchContext):
        num_drones = int(LaunchConfiguration("num_drones").perform(context))

        if num_drones != 2:
            return [
                Node(
                    package="joy",
                    executable="joy_node",
                    name="freya_joy_node",
                    output="screen",
                    parameters=[
                        {
                            "deadzone": 0.15,
                            "autorepeat_rate": 100.0,
                        }
                    ],
                    remappings=[
                        ("/joy", "/freya/joy"),
                    ],
                )
            ]

        elif num_drones == 2:
            return [
                Node(
                    package="joy",
                    executable="joy_node",
                    name="joy_node",
                    output="screen",
                    parameters=[
                        {
                            "deadzone": 0.15,
                            "autorepeat_rate": 100.0,
                            "device_name": "Xbox 360 Controller",
                        }
                    ],
                    remappings=[
                        ("/joy", "/freya/joy"),
                    ],
                )
            ]

    return LaunchDescription(
        [
            num_drones_arg,
            set_env_var,
            set_warn_color,
            OpaqueFunction(function=include_joy_node),
            joystick_interface_launch,
            thrust_allocator_asv_launch,
        ]
    )
