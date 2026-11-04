"""Запуск исследования клеток со SLAM и Nav2/MPPI; gui:=false — без окон."""

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
    pkg = get_package_share_directory("solution")
    nav = get_package_share_directory("nav2_bringup")
    params = RewrittenYaml(
        source_file=os.path.join(pkg, "config", "nav2_params.yaml"),
        param_rewrites={"yaw_goal_tolerance": "3.14159"},
        convert_types=True,
    )
    evaluation_params = RewrittenYaml(
        source_file=os.path.join(pkg, "config", "nav2_params.yaml"),
        root_key="exploration",
        param_rewrites={"use_sim_time": "true"},
        convert_types=True,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "slam_toolbox.yaml"),
                    {"use_sim_time": True},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                namespace="exploration",
                output="screen",
                parameters=[evaluation_params],
                remappings=[("map", "/map"), ("scan", "/scan")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                namespace="exploration",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["planner_server"],
                    }
                ],
            ),
            Node(
                package="solution",
                executable="cell_explorer_node.py",
                name="cell_explorer",
                output="screen",
                parameters=[os.path.join(pkg, "config", "cell_explorer.yaml")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg, "config", "cell_explorer.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )
