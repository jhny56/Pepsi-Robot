from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='Blackband_detector',  # Adjusted path for the Python node
            name='Blackband_detector',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }],
        ),
        Node(
            package='perception',
            executable='pepsican_detector',  # Adjusted path for the Python node
            name='pepsican_detector',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }],
        ),
        Node(
            package='perception',
            executable='qrcode_detector',  # Adjusted path for the Python node
            name='qrcode_detector',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }],
        ),
        Node(
            package='perception',
            executable='imagepublisher',  # Adjusted path for the Python node
            name='imagepublisher',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }],
        ),

    ])
