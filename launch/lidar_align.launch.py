from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    results_dir = os.path.join('/tmp', 'lidar_align_results')
    os.makedirs(results_dir, exist_ok=True)

    default_pointcloud_path = os.path.join(
        results_dir, f'lidar_points_{timestamp}.ply')
    default_calibration_path = os.path.join(
        results_dir, f'calibration_{timestamp}.txt')

    return LaunchDescription([
        DeclareLaunchArgument('bag_file', default_value=''),
        DeclareLaunchArgument('transforms_from_csv', default_value='false'),
        DeclareLaunchArgument('csv_file', default_value=''),
        DeclareLaunchArgument(
            'output_pointcloud_path', default_value=default_pointcloud_path),
        DeclareLaunchArgument(
            'output_calibration_path', default_value=default_calibration_path),
        DeclareLaunchArgument('local', default_value='false'),
        DeclareLaunchArgument('inital_guess', default_value='0.0, 0.0, 0.0, 0.0, 0.0, 0.0'),
        Node(
            package='lidar_align',
            executable='lidar_align_node',
            name='lidar_align',
            output='screen',
            parameters=[{
                'input_bag_path': LaunchConfiguration('bag_file'),
                'input_csv_path': LaunchConfiguration('csv_file'),
                'output_pointcloud_path': LaunchConfiguration(
                    'output_pointcloud_path'),
                'output_calibration_path': LaunchConfiguration(
                    'output_calibration_path'),
                'transforms_from_csv': LaunchConfiguration(
                    'transforms_from_csv'),
                'local': LaunchConfiguration('local'),
                'inital_guess': LaunchConfiguration('inital_guess'),
            }],
        ),
    ])
