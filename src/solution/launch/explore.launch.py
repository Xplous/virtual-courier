import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('solution')
    nav = get_package_share_directory('nav2_bringup')
    params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        param_rewrites={'yaw_goal_tolerance': '3.14159'}, convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml'), {'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'params_file': params,
                              'autostart': 'true', 'use_composition': 'False'}.items()),
        Node(
            package='solution', executable='exploration_map_node.py', name='exploration_map',
            output='screen', parameters=[{'use_sim_time': True}]),
        Node(
            package='explore_lite', executable='explore', name='explore_node', output='screen',
            parameters=[os.path.join(pkg, 'config', 'explore.yaml')]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'explore.rviz')],
            parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('gui'))),
    ])
