import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share_bringup = get_package_share_directory('car_bringup')
    pkg_share_description = get_package_share_directory('car_description')
    pkg_share_localization = get_package_share_directory('car_localization')



    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    #done
    default_model_path = os.path.join(pkg_share_description, 'urdf', 'car.sdf')
    default_rviz_config_path = os.path.join(pkg_share_description, 'rviz', 'urdf_config.rviz')
    # world_path = os.path.join(pkg_share_sambot, 'world', 'my_world.sdf')
    #Done   
    bridge_config_path = os.path.join(pkg_share_bringup, 'config', 'gazebo_bridge.yaml')
    
    #Done
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    #done
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

   #done
    gz_server = GzServer(
        world_sdf_file='model.sdf',
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    #done
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/depth_camera/image'],
    )
    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/depth_camera/depth_image'],
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'my_robot',
            'z': '0.65',
        }.items(),
    )

    #done
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share_localization, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        camera_bridge_image,
        camera_bridge_depth,
        spawn_entity,
        # robot_localization_node,
    ])