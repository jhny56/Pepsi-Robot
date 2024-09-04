from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_behavior_tree',
            executable='behavior_tree_node',
            name='behavior_tree_node',
            output='screen'
        ),
        Node(
            package='robot_behavior_tree',
            executable='initNode',
            name='initNode',
            output='screen'
        )
    ])