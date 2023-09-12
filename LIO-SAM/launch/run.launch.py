import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='log',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path]),
                "use_tf_static" : True}]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='log'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='log'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='log'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='log'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "-1.57", "0", "3.14", "odom", "lidar_odom"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-2", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["6.5", "7.47", "0", "0", "0", "3.14", "map_decision", "map"]
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=["4.3", "7.47", "0", "0", "0", "3.14", "map_decision", "map"]
        # ),
        # Node(
        # package="tf2_ros",
        # executable="static_transform_publisher",
        # output="log" ,
        # arguments=["0", "0", "0", "0", "0", "3.14", "odom", "lidar_odom"]
        # ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # )
    ])
