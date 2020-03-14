from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='infant',
            node_executable='ear',
        ),
        Node(
            package='infant',
            node_executable='eye',
        ),
        # Node(
        #     package='infant',
        #     node_executable='touch',
        # ),
        # Node(
        #     package='infant',
        #     node_executable='pulse',
        # ),
        Node(
            package='infant',
            node_executable='state_server',
        ),
        Node(
            package='infant',
            node_executable='visualizer',
        ),
    ])
