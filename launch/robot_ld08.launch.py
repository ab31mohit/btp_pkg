import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_NAMESPACE = os.environ['TURTLEBOT3_NAMESPACE']  # get the namespace for this robot from the environment variable

    return LaunchDescription([
        Node(
            package='ld08_driver',
            node_executable='ld08_driver',
            node_name='ld08_driver',
            node_namespace='burger_1',  # <--- CHANGE THIS to your desired robot namespace
            output='screen'),
    ])

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import LogInfo
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     port = LaunchConfiguration('port', default='/dev/ttyUSB0')

#     frame_id = LaunchConfiguration('frame_id', default='laser')

#     return LaunchDescription([

#         DeclareLaunchArgument(
#             'port',
#             default_value=port,
#             description='Specifying usb port to connected lidar'),

#         DeclareLaunchArgument(
#             'frame_id',
#             default_value=frame_id,
#             description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

#         Node(
#             package='hls_lfcd_lds_driver',
#             node_executable='hlds_laser_publisher',
#             node_name='hlds_laser_publisher',
#             node_namespace='tb3_0',  # <------------------- ADD THIS!
#             parameters=[{'port': port, 'frame_id': frame_id}],
#             output='screen'),
#     ])