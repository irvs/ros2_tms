import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
  return LaunchDescription([
      launch_ros.actions.Node(
        package='tms_db_manager', node_executable='tms_db_reader', output='screen'),
      launch_ros.actions.Node(
        package='tms_db_manager', node_executable='tms_db_writer', output='screen')
  ])
   