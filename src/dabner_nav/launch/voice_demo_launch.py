import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy

def generate_launch_description():
    logger = rclpy.logging.get_logger('navigation_launch')


    dabner_nav_path = get_package_share_directory('dabner_nav')
    
    jec_nav_node = Node(
            package='dabner_nav',
            executable='jec_nav',
            name='jec_nav',
            output='screen',
    )

    speech_node = Node(
            package='dabner_nav',
            executable='speech_recognition',
            name='speech_recognition',
            output='screen',
    )

    logic_node = Node(
            package='dabner_nav',
            executable='nav_logic',
            name='nav_logic',
            output='screen',
    )

    return LaunchDescription([
        jec_nav_node,
        speech_node,
        logic_node,
    ])
