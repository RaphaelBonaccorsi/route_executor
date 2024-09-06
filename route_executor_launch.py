import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the correct PX4-Autopilot directory
    px4_autopilot_dir = '/home/harpia/PX4-Autopilot'  # Replace with the correct absolute path
    
    # Get the package share directory for route_executor
    package_share = get_package_share_directory('route_executor')

    return LaunchDescription([
        Node(
            package='route_executor',
            executable='route_executor',
            output='screen'
        )
    ])
