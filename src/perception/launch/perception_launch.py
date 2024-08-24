from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='perception',
        #     executable='Blackband_detector',  # Adjusted path for the Python node
        #     name='Blackband_detector',
        #     output='screen',
        #     parameters=[{
        #         # Add parameters here if needed
        #     }],
        # ),
        # Node(
        #     package='perception',
        #     executable='Blue_detector',  # Adjusted path for the Python node
        #     name='Blue_detector',
        #     output='screen',
        #     parameters=[{
        #         # Add parameters here if needed
        #     }],
        # ),
        Node(
            package='perception',
            executable='White_detector',  # Adjusted path for the Python node
            name='White_detector',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }],
        ),
    ])
