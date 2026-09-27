"""Visualise le bras dans RViz2 avec des sliders (joint_state_publisher_gui).

    ros2 launch robot_description view_robot.launch.py
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
    share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(share, 'urdf', 'arm6.urdf.xacro')
    rviz_file = os.path.join(share, 'rviz', 'robot.rviz')

    config = LaunchConfiguration('config')
    use_gui = LaunchConfiguration('use_gui')

    # Le xacro relit le YAML : modifier robot.yaml suffit pour changer les longueurs.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' config:=', config]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'robot.yaml'),
            description='Fichier de paramètres du robot'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Sliders joint_state_publisher_gui'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file], output='screen'),
    ])
