import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("Docking_pkg")
    config_file = os.path.join(pkg_dir, "config", "docking_planner.yaml")
    rviz_config = os.path.join(pkg_dir, "config", "docking.rviz")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    target_speed = LaunchConfiguration("target_speed")

    simulator = Node(
        package="Docking_pkg",
        executable="demo_sim_node",
        name="demo_sim_node",
        output="screen",
        parameters=[
            {
                "sim.ego_x": 0.0,
                "sim.ego_y": 0.0,
                "sim.ego_yaw": 0.0,
                "sim.target_x": initial_x,
                "sim.target_y": initial_y,
                "sim.target_yaw": 0.0,
                "sim.target_speed": target_speed,
                "sim.target_yaw_rate": 0.0,
                "sim.wheel_radius": 0.254,
                "sim.wheel_track": 0.60,
                "sim.publish_hz": 50.0,
                "sim.measurement_noise_std": 0.008,
                "sim.velocity_noise_std": 0.01,
                "sim.qr_pose_noise_std": 0.005,
                "sim.camera_x": 0.35,
                "sim.camera_y": 0.0,
                "sim.camera_z": 0.20,
                "sim.camera_yaw": 0.0,
                "sim.tag_x": -0.26,
                "sim.tag_y": 0.0,
                "sim.tag_z": 0.05,
                "sim.tag_yaw": 0.0,
                "sim.body_length": 0.52,
                "sim.body_width": 0.45,
                "sim.body_height": 0.40,
                "sim.cone_bus": 0.08,
            }
        ],
    )

    adapter = Node(
        package="Docking_pkg",
        executable="qr_target_adapter_node",
        name="qr_target_adapter_node",
        output="screen",
        parameters=[
            {
                "planning_frame": "ego_base",
                "qr_tag_frame": "qr_tag",
                "target_base_frame": "target_base",
                "publish_twist": False,
                "velocity_filter_alpha": 0.35,
            }
        ],
    )

    planner = Node(
        package="Docking_pkg",
        executable="docking_planner_node",
        name="docking_planner_node",
        output="screen",
        parameters=[config_file],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "initial_x", default_value="7.0",
            description="目标车相对自车的初始 X 坐标，单位 m"),
        DeclareLaunchArgument(
            "initial_y", default_value="1.8",
            description="目标车相对自车的初始 Y 坐标，单位 m"),
        DeclareLaunchArgument(
            "target_speed", default_value="0.35",
            description="目标车沿 odom +X 的直线速度，单位 m/s"),
        simulator,
        adapter,
        planner,
        rviz,
    ])
