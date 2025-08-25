from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_line',
            executable='follow_line_executable',
            output='screen',
            emulate_tty=True,
        ),
    ])