"""SIMULATION seule (aucun matériel) : tout sur une machine, le STM32 est émulé par sim_robot_node.

    ros2 launch robot_bringup sim.launch.py                    # cœur + simulateur + IHM
    ros2 launch robot_bringup sim.launch.py gui:=false rviz:=true
    ros2 launch robot_bringup sim.launch.py start_ready:=false # tester Activer + Homing
    ros2 launch robot_bringup sim.launch.py vision:=true camera_source:=opencv   # avec une webcam
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    desc = get_package_share_directory('robot_description')
    params = os.path.join(bringup, 'config', 'params.yaml')
    xacro_file = os.path.join(desc, 'urdf', 'arm6.urdf.xacro')
    rviz_file = os.path.join(desc, 'rviz', 'robot.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Lance l\'IHM PySide6'),
        DeclareLaunchArgument('rviz', default_value='false', description='Lance RViz2 (+ robot fantôme)'),
        DeclareLaunchArgument('start_ready', default_value='true',
                              description='true : robot simulé déjà référencé ; false : démarre désactivé'),
        DeclareLaunchArgument('vision', default_value='false'),
        DeclareLaunchArgument('camera_source', default_value='opencv', description='opencv | topic'),
        DeclareLaunchArgument('detector', default_value='aruco'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'core.launch.py')),
            launch_arguments={'mode': 'simulation'}.items()),

        Node(package='robot_control', executable='sim_robot_node', output='screen',
             parameters=[params, {'start_ready': LaunchConfiguration('start_ready')}]),

        Node(package='robot_vision', executable='vision_node', output='screen',
             condition=IfCondition(LaunchConfiguration('vision')),
             parameters=[params, {'source': LaunchConfiguration('camera_source'),
                                  'detector': LaunchConfiguration('detector')}]),

        Node(package='robot_gui', executable='gui', output='screen',
             condition=IfCondition(LaunchConfiguration('gui'))),

        # RViz2 + robot fantôme (consigne)
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='ghost',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'robot_description': robot_description, 'frame_prefix': 'ghost/'}],
             remappings=[('joint_states', '/joint_commands')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'ghost/base_link']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
