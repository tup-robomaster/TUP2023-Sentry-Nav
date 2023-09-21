import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config', 'mid360.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')
    xacro_path = os.path.join(package_path, 'config', 'robot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    use_rviz = True

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time}],
        output='both'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro', ' ', xacro_path]),
            "use_tf_static" : True}]
    )

    tf_adapter_1 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="log" ,
    arguments=["0", "0", "0", "0", "0", "0", "lidar_odom", "camera_init"]
    )
    tf_adapter_2 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="log" ,
    arguments=["0", "0", "0", "-1.57", "0", "3.14", "odom", "lidar_odom"]
    )
    tf_adapter_3 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="log" ,
    arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    if use_rviz:
        ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(tf_adapter_1)
    ld.add_action(tf_adapter_2)
    ld.add_action(tf_adapter_3)

    return ld
