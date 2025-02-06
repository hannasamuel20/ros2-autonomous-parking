import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the path to the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory('car_description'),
        'rviz',
        'urdf_config.rviz'
    )

    # Define the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        joint_state_publisher_node,
        rviz_node
    ])
