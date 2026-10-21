import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("Docking_pkg")
    config_file = os.path.join(pkg_dir, "config", "docking_planner.yaml")

    planner = Node(
        package="Docking_pkg",
        executable="docking_planner_node",
        name="docking_planner_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([planner])
