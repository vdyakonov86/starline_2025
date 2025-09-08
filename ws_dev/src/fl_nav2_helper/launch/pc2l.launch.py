from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument(
      name="scanner", default_value="scanner",
      description="Namespace for sample topics"
    ),
    Node(
      package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
      remappings=[('cloud_in', '/livox/lidar'), ('scan', '/lidar_scan')],
      parameters=[{
        'target_frame': 'livox',
        'transform_tolerance': 0.01,
        'min_height': -1.0,
        'max_height': 1.5,
        'angle_min': -3.141592654,
        'angle_max': 3.141592654,
        'angle_increment': 0.003141592,
        'scan_time': 0.03333,
        'range_min': 0.5,
        'range_max': 40.0,
        'use_inf': True,
        'inf_epsilon': 1.0
      }],
      name='pointcloud_to_laserscan'
    )
  ])

