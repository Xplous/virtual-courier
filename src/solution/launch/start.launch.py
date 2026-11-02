#!/usr/bin/env python3
"""Точка входа решения. Организаторы запускают именно этот файл:

    ros2 launch solution start.launch.py gui:=false

Аргументы:
    gui   — разрешены ли окна (rviz, cv2.imshow); на прогоне организаторов false
    slam  — запустить slam_toolbox (строит /map и TF map→odom)
    nav   — запустить Nav2 (требует slam:=true или свою локализацию)
    explore — исследование по лидару с памятью пути; по умолчанию включает slam
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'solution'


def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)
    gui = LaunchConfiguration('gui')
    slam = LaunchConfiguration('slam')
    nav = LaunchConfiguration('nav')
    explore = LaunchConfiguration('explore')
    use_sim_time = {'use_sim_time': True}

    courier = Node(
        package=PACKAGE,
        executable='courier_node.py',
        name='courier',
        output='screen',
        parameters=[use_sim_time, {'gui': gui}],
        condition=UnlessCondition(nav),
    )

    explorer = Node(
        package=PACKAGE,
        executable='explorer_node.py',
        name='explorer',
        output='screen',
        parameters=[use_sim_time],
        condition=IfCondition(explore),
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml'), use_sim_time],
        condition=IfCondition(slam),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
        }.items(),
        condition=IfCondition(nav),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'config', 'courier.rviz')],
        parameters=[use_sim_time],
        condition=IfCondition(gui),
    )

    # При остановке решения робот должен остановиться.
    stop_robot = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', '{}'])]))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('slam', default_value=explore),
        DeclareLaunchArgument('nav', default_value='false'),
        slam_toolbox,
        GroupAction(condition=UnlessCondition(explore), actions=[courier, navigation]),
        explorer,
        rviz,
        stop_robot,
    ])
