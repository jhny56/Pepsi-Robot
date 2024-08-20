from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='initNode',
            name='init_node',
            output='screen',
            parameters=[{
                # Add any parameters you want to set for the node here
            }]
        ),
    ])
