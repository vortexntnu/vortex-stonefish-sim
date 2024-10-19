from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vortex_sim_interface',
            executable='vortex_sim_interface',
            name='vortex_sim_interface',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])