import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': './pddl/domain.pddl'}.items()
        )
    
    route_executor_node = Node(
        package='route_executor',
        executable='route_executor',
        output='screen'
    )

    pddl_reader_node = Node(
        package='pddl_reader',
        executable='pddl_reader',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(route_executor_node)
    ld.add_action(pddl_reader_node)
 

    return ld