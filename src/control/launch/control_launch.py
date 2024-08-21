from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='initNode',
            name='init_node',
            output='screen',
            parameters=[{
                # Add any parameters you want to set for the node here
            }]
        ),
        Node(
            package='control',
            executable='gripper_action_server',
            name='gripper_action_server',
            output='screen',
            parameters=[{
                # Add any parameters you want to set for the node here
            }]
        ),
        Node(
            package='control',
            executable='gripper_action_client',
            name='gripper_action_client',
            output='screen',
            parameters=[{
                # Add any parameters you want to set for the node here
            }]
        ),
    ])
