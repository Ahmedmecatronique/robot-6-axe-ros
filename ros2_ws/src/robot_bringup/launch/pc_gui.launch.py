"""PC portable — IHM de supervision (jumeau numérique). Ne lance AUCUN nœud de calcul lourd.

    ros2 launch robot_bringup pc_gui.launch.py            # IHM seule
    ros2 launch robot_bringup pc_gui.launch.py rviz:=true # + RViz2 (robot mesuré + fantôme de consigne)

Prérequis : même ROS_DOMAIN_ID que le Raspberry Pi, même réseau.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('robot_description')
    xacro_file = os.path.join(desc, 'urdf', 'arm6.urdf.xacro')
    rviz_file = os.path.join(desc, 'rviz', 'robot.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    use_rviz = LaunchConfiguration('rviz')

    # Robot "fantôme" de la consigne : mêmes liens préfixés « ghost/ », alimenté par /joint_commands
    ghost = GroupAction(condition=IfCondition(use_rviz), actions=[
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='ghost',
             parameters=[{'robot_description': robot_description, 'frame_prefix': 'ghost/'}],
             remappings=[('joint_states', '/joint_commands')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'ghost/base_link']),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file], output='screen'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Lance aussi RViz2'),
        Node(package='robot_gui', executable='gui', output='screen'),
        ghost,
    ])
