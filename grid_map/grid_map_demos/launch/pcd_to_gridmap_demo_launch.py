import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')

    # Declare launch configuration variables that can access the launch arguments values
    param_file = LaunchConfiguration('param_file')
    visualization_config_file = LaunchConfiguration('visualization_config')

    # Declare launch arguments
    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'pcd_to_gridmap_demo.yaml'),
        description='Full path to the config file to use')

    # Declare launch arguments
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'pcd_grid_rviz.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'grid_map_demo.rviz'),
        description='Full path to the RVIZ config file to use')
    
    pcd_to_gridmap_demo_node = Node(
        package='grid_map_demos',
        executable='pcd_to_gridmap_demo',
        name='pcd_to_gridmap',
        parameters=[param_file,
                    {
                    "config_file_path" : os.path.join(grid_map_demos_dir, 'config', 'pcl_grid_config.yaml')
                    }
                ],
        output="screen",
        respawn=True
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_visualization_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(pcd_to_gridmap_demo_node)
    ld.add_action(grid_map_visualization_node)
    return ld
