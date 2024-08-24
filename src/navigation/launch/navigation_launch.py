from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('navigation'),  # The package name
        'config',  # Reference the config directory
        'params.yaml'  # The YAML file inside the config directory
    )

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
        Node(
            package='navigation',
            executable='Qr_navigation_node',
            name='Qr_navigation_node',
            output='screen',
            parameters=[params_file]
        ),
    ])
