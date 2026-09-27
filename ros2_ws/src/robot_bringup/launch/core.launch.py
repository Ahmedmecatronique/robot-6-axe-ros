"""Cœur commun (Raspberry Pi ou simulation) : URDF, FK, IK, trajectoire, sécurité, saisie.

Argument :  mode:=real | simulation   (transmis à safety_node ; publié sur /robot_mode)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc = get_package_share_directory('robot_description')
    params = os.path.join(bringup, 'config', 'params.yaml')
    xacro_file = os.path.join(desc, 'urdf', 'arm6.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='simulation', description='real | simulation'),
        DeclareLaunchArgument('rosbridge', default_value='true',
                              description='WebSocket rosbridge (port 9090) pour une IHM Windows sans ROS 2'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='robot_kinematics', executable='fk_node', parameters=[params], output='screen'),
        Node(package='robot_kinematics', executable='ik_node', parameters=[params], output='screen'),
        Node(package='robot_kinematics', executable='trajectory_node', parameters=[params], output='screen'),
        Node(package='robot_control', executable='safety_node',
             parameters=[params, {'mode': mode}], output='screen'),
        Node(package='robot_control', executable='pick_node', parameters=[params], output='screen'),
        Node(package='robot_control', executable='bag_node', output='screen'),
        # Pont WebSocket : l'IHM Windows (roslibpy) s'y connecte, aucune installation de ROS 2 côté PC.
        # ATTENTION : aucune authentification -> à limiter au réseau local / pare-feu (voir docs/INSTALLATION.md).
        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             condition=IfCondition(LaunchConfiguration('rosbridge')),
             parameters=[{'port': LaunchConfiguration('rosbridge_port'), 'address': ''}]),
    ])
