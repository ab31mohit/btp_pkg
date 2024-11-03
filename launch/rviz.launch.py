import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('btp_pkg'),
        'rviz',
        'robot_config.rviz')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    
    # Declare nodes
    # Trajectory node
    trajectory_node_cmd = Node(
    package='btp_pkg',
    executable='trajectory_node.py',  # Replace with your first node's executable name
    name='trajectory_node',        # Replace with desired node name
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
    )

    # Rviz2 node
    rviz_node_cmd = Node(
        package='rviz2',
        executable='rviz2',  # Replace with your second node's executable name
        name='rviz2',        # Replace with desired node name
        arguments=['-d', rviz_config_dir],
        output='screen'
        # parameters=[{'use_sim_time': use_sim_time}]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(trajectory_node_cmd)
    ld.add_action(rviz_node_cmd)
    
    return ld
    # return LaunchDescription([
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', rviz_config_dir],
    #         output='screen'),
    # ])