import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROBOT_NAMESPACE = os.environ['TURTLEBOT3_NAMESPACE']  # get the namespace for this robot from the environment variable

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # remap topics to your desired robot namespace
    remappings = [('/tf', '/burger_1/tf'),
                  ('/tf_static', '/burger_1/tf_static')]
    
    # print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # namespace=ROBOT_NAMESPACE,
            namespace='burger_1',  # <--- CHANGE THIS to your desired robot namespace
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            remappings=remappings)
    ])
