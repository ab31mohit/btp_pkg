import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_NAMESPACE = os.environ['TURTLEBOT3_NAMESPACE']  # get the namespace for this robot from the environment variable

    return LaunchDescription([
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            namespace='burger_1',  # <--- CHANGE THIS to your desired robot namespace
            output='screen'),
    ])
