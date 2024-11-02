#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    btp_pkg_dir = get_package_share_directory('btp_pkg')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the nodes
    first_node_cmd = Node(
        package='btp_pkg',
        executable='trajectory_node',  # Replace with your first node's executable name
        name='trajectory_node',        # Replace with desired node name
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    second_node_cmd = Node(
        package='btp_pkg',
        executable='waypoints_node',  # Replace with your second node's executable name
        name='waypoints_node',        # Replace with desired node name
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(first_node_cmd)
    ld.add_action(second_node_cmd)

    return ld