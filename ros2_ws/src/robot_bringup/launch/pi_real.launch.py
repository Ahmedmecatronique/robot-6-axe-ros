"""Raspberry Pi 4 — robot RÉEL : cœur + agent micro-ROS (liaison STM32) + vision.

    ros2 launch robot_bringup pi_real.launch.py serial_dev:=/dev/ttyACM0 camera_source:=picamera2

Le STM32 (micro-ROS) publie /joint_states et /stm32/status, et s'abonne à /joint_commands,
/emergency_stop, /stm32/command, /gripper_command. L'agent fait le pont UART <-> DDS.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    params = os.path.join(bringup, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyACM0',
                              description='Port série vers le STM32 (ST-LINK : /dev/ttyACM0 ; UART câblé : /dev/ttyAMA0)'),
        DeclareLaunchArgument('baud', default_value='921600'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('camera_source', default_value='picamera2',
                              description='picamera2 | opencv | topic'),
        DeclareLaunchArgument('detector', default_value='aruco', description='aruco | hsv'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
            launch_arguments={'mode': 'real'}.items()),

        # Agent micro-ROS : pont série <-> DDS (installé séparément, voir docs/INSTALLATION.md)
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
                 '--dev', LaunchConfiguration('serial_dev'), '-b', LaunchConfiguration('baud')],
            output='screen', name='micro_ros_agent'),

        Node(package='robot_vision', executable='vision_node', output='screen',
             condition=IfCondition(LaunchConfiguration('vision')),
             parameters=[params, {'source': LaunchConfiguration('camera_source'),
                                  'detector': LaunchConfiguration('detector')}]),
    ])
