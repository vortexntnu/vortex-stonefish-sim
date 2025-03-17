from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


class ConcatenateSubstitutions(Substitution):
    def __init__(self, *substitutions):
        self.substitutions = substitutions

    def perform(self, context):
        return "".join([sub.perform(context) for sub in self.substitutions])


def generate_launch_description():
    # Get the directories of the involved packages
    vortex_stonefish_sim_dir = get_package_share_directory("stonefish_sim")
    stonefish_ros2_dir = get_package_share_directory("stonefish_ros2")

    simulation_data_default = PathJoinSubstitution([vortex_stonefish_sim_dir, "data"])

    simulation_data_arg = DeclareLaunchArgument(
        "simulation_data",
        default_value=simulation_data_default,
        description="Path to the simulation data folder",
    )

    scenario_desc_arg = DeclareLaunchArgument(
        "task",
        default_value=PathJoinSubstitution("orca_no_gpu"),
        description="Path to the scenario file",
        choices=["orca_no_gpu", "freya_no_gpu"],
    )

    scenario_desc_resolved = PathJoinSubstitution(
        [
            vortex_stonefish_sim_dir,
            "scenarios",
            ConcatenateSubstitutions(
                LaunchConfiguration("task"), TextSubstitution(text=".scn")
            ),
        ]
    )

    include_stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [stonefish_ros2_dir, "/launch/stonefish_simulator_nogpu.launch.py"]
        ),
        launch_arguments={
            "simulation_data": LaunchConfiguration("simulation_data"),
            "scenario_desc": scenario_desc_resolved,
        }.items(),
    )

    return LaunchDescription(
        [simulation_data_arg, scenario_desc_arg, include_stonefish_launch]
    )
