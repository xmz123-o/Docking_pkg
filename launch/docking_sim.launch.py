import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("Docking_pkg")
    config_file = os.path.join(pkg_dir, "config", "docking_planner.yaml")
    rviz_config = os.path.join(pkg_dir, "config", "docking.rviz")

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
                "sim.target_x": 7.0,
                "sim.target_y": 0.0,
                "sim.target_yaw": 0.0,
                "sim.target_speed": 0.35,
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

    return LaunchDescription([simulator, adapter, planner])
