import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('solution')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml'), {'use_sim_time': True}],
            output='screen'),
        Node(
            package='solution', executable='tremaux_node.py', name='tremaux',
            parameters=[{'use_sim_time': True}], output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'tremaux.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('gui'))),
    ])
