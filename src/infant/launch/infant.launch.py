from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='infant',
            executable='ear',
        ),
        Node(
            package='infant',
            executable='eye',
        ),
        Node(
            package='infant',
            executable='touch',
        ),
        Node(
            package='infant',
            executable='pulse',
        ),
        Node(
            package='infant',
            executable='state_server',
        ),
        Node(
            package='infant',
            executable='visualizer',
        ),
    ])
