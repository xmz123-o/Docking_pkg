"""真实 ArUco 目标位姿 + 仿真自车轮速/横摆角的 RViz 闭环。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    docking_share = get_package_share_directory("Docking_pkg")
    aruco_share = get_package_share_directory("aruco_det")

    rviz_config = os.path.join(docking_share, "config", "docking.rviz")
    default_camera_yaml = os.path.join(aruco_share, "config", "usb_camera.yaml")
    aruco_launch = os.path.join(aruco_share, "launch", "usb_aruco.launch.py")

    video_device = LaunchConfiguration("video_device")
    camera_param_path = LaunchConfiguration("camera_param_path")
    marker_length = LaunchConfiguration("marker_length")
    primary_id = LaunchConfiguration("primary_id")
    start_camera = LaunchConfiguration("start_camera")
    show_camera_view = LaunchConfiguration("show_camera_view")
    pixel_format = LaunchConfiguration("pixel_format")
    image_qos_reliability = LaunchConfiguration("image_qos_reliability")
    ar_pose_stale_timeout = LaunchConfiguration("ar_pose_stale_timeout")
    publish_aruco_tf = LaunchConfiguration("publish_aruco_tf")
    target_speed = LaunchConfiguration("target_speed")
    target_world_yaw = LaunchConfiguration("target_world_yaw")
    mounting_yaw_offset = LaunchConfiguration("mounting_yaw_offset")
    prefer_velocity_heading = LaunchConfiguration("prefer_velocity_heading")
    live_target_update = LaunchConfiguration("live_target_update")
    camera_mount_x = LaunchConfiguration("camera_mount_x")
    camera_mount_y = LaunchConfiguration("camera_mount_y")
    camera_mount_z = LaunchConfiguration("camera_mount_z")
    use_rviz = LaunchConfiguration("rviz")

    camera_and_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_launch),
        launch_arguments={
            "start_camera": start_camera,
            "start_detector": "true",
            "view": show_camera_view,
            "video_device": video_device,
            "pixel_format": pixel_format,
            "image_qos_reliability": image_qos_reliability,
            "camera_param_path": camera_param_path,
            "aruco_length": marker_length,
            "primary_id": primary_id,
            "publish_tf": publish_aruco_tf,
        }.items(),
    )

    # 这里只把 ArUco 的光学坐标定义转换为规划器使用的 camera_link 轴。
    qr_adapter = Node(
        package="Docking_pkg",
        executable="aruco_to_docking_adapter.py",
        name="aruco_to_docking_adapter",
        output="screen",
        parameters=[
            {
                "input_topic": "/aruco/pose",
                "output_topic": "/qr/tag_measurement",
                "yaw_output_topic": "/qr/tag_yaw_camera",
                "mounting_yaw_offset": mounting_yaw_offset,
                "output_frame": "camera_link",
                "axis_mode": "optical_to_camera_link",
                "stale_timeout": ar_pose_stale_timeout,
                "min_range": 0.05,
                "max_range": 30.0,
            }
        ],
    )

    target_bridge = Node(
        package="Docking_pkg",
        executable="hybrid_target_bridge.py",
        name="hybrid_target_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": "/qr/tag_measurement",
                "output_topic": "/qr/tag_position",
                "ego_pose_topic": "/ego/pose",
                "reset_topic": "/qr/hybrid/reanchor",
                "marker_topic": "/hybrid_target/markers",
                "pose_topic": "/hybrid_target/pose",
                "twist_topic": "/hybrid_target/twist",
                "yaw_input_topic": "/qr/tag_yaw_camera",
                "use_measured_target_heading": True,
                "target_heading_filter_alpha": 0.25,
                "heading_timeout": 1.0,
                "heading_wait_timeout": 0.25,
                "heading_jump_tolerance": 0.70,
                "prefer_velocity_heading": prefer_velocity_heading,
                "velocity_heading_min_speed": 0.06,
                "velocity_heading_filter_alpha": 0.30,
                "velocity_heading_required_samples": 5,
                "velocity_heading_candidate_tolerance": 0.35,
                "target_length": 0.52,
                "target_width": 0.45,
                "target_height": 0.40,
                "tag_on_rear_face": True,
                "output_frame": "camera_link",
                "camera_x": 0.35,
                "camera_y": 0.0,
                "camera_yaw": 0.0,
                "tag_x": -0.26,
                "tag_y": 0.0,
                "tag_yaw": 0.0,
                "target_world_yaw": target_world_yaw,
                "target_speed": target_speed,
                "live_target_update": live_target_update,
                "position_filter_alpha": 0.65,
                "velocity_filter_alpha": 0.20,
                "prediction_timeout": 0.35,
                "max_target_speed": 1.5,
                "publish_hz": 30.0,
            }
        ],
    )

    # 真实目标来自 ArUco。此节点仅模拟自车由 wheel_cmd 积分得到的
    # /ego/twist、/ego/yaw、/ego/pose 以及 odom -> ego_base TF。
    ego_simulator = Node(
        package="Docking_pkg",
        executable="demo_sim_node",
        name="demo_sim_node",
        output="screen",
        parameters=[
            {
                "sim.publish_qr": False,
                "sim.publish_simulated_target": False,
                "sim.ego_x": 0.0,
                "sim.ego_y": 0.0,
                "sim.ego_yaw": 0.0,
                "sim.target_speed": 0.0,
                "sim.target_yaw_rate": 0.0,
                "sim.wheel_radius": 0.254,
                "sim.wheel_track": 0.60,
                "sim.publish_hz": 50.0,
                "sim.measurement_noise_std": 0.001,
                "sim.velocity_noise_std": 0.005,
                "sim.camera_x": 0.35,
                "sim.camera_y": 0.0,
                "sim.camera_z": 0.20,
                "sim.camera_yaw": 0.0,
                "sim.body_length": 0.52,
                "sim.body_width": 0.45,
                "sim.body_height": 0.40,
                "sim.cone_bus": 0.08,
            }
        ],
    )

    # camera_link -> camera_optical_frame 只差一个固定坐标轴旋转。
    # 混合模式不使用间歇的 aruco_0 TF 做可视化。
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ego_to_camera_optical_tf",
        arguments=[
            "--x", camera_mount_x,
            "--y", camera_mount_y,
            "--z", camera_mount_z,
            "--roll", "-1.57079632679",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", "ego_base",
            "--child-frame-id", "camera_optical_frame",
        ],
    )

    planner = Node(
        package="Docking_pkg",
        executable="odom_docking_controller.py",
        name="odom_docking_controller",
        output="screen",
        parameters=[
            {
                "desired_center_distance": 0.52,
                "align_heading_tolerance": 0.035,
                "align_lateral_tolerance": 0.02,
                "dock_distance_tolerance": 0.02,
                "dock_lateral_tolerance": 0.025,
                "final_press_speed": 0.035,
                "release_distance": 0.05,
                "release_lateral": 0.04,
                "approach_speed": 0.40,
                "final_speed": 0.20,
                "target_motion_threshold": 0.05,
                "max_accel": 0.60,
                "max_decel": 0.80,
                "heading_gain": 1.20,
                "bearing_gain": 0.80,
                "yaw_rate_gain": 0.30,
                "max_angular": 0.60,
                "axis_correction_gain": 1.20,
                "path_tangent_scale": 0.20,
                "lookahead_min": 0.32,
                "lookahead_max": 0.65,
                "heading_preview_gain": 0.55,
                "terminal_pose_distance": 0.15,
                "wheel_radius": 0.254,
                "wheel_track": 0.60,
                "max_wheel_rad_s": 8.0,
                "control_period": 0.05,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="USB 相机设备节点",
            ),
            DeclareLaunchArgument(
                "camera_param_path",
                default_value=default_camera_yaml,
                description="相机内参 YAML；必须与当前相机和分辨率匹配",
            ),
            DeclareLaunchArgument(
                "marker_length",
                default_value="0.08",
                description="ArUco 黑色方块的边长，单位 m",
            ),
            DeclareLaunchArgument(
                "primary_id",
                default_value="0",
                description="目标车辆使用的 ArUco ID",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="是否由本 launch 启动 usb_cam",
            ),
            DeclareLaunchArgument(
                "show_camera_view",
                default_value="false",
                description="是否打开 ArUco 检测图像窗口",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="yuyv",
                description="USB 相机像素格式：mjpeg 或 yuyv",
            ),
            DeclareLaunchArgument(
                "image_qos_reliability",
                default_value="best_effort",
                description="图像 QoS：实时调试推荐 best_effort",
            ),
            DeclareLaunchArgument(
                "ar_pose_stale_timeout",
                default_value="1.5",
                description="混合仿真允许的 ArUco 位姿最大延迟，单位 s",
            ),
            DeclareLaunchArgument(
                "publish_aruco_tf",
                default_value="false",
                description="是否发布间歇的 camera_optical_frame -> aruco_0 TF",
            ),
            DeclareLaunchArgument(
                "target_speed",
                default_value="0.0",
                description="目标锁定后模拟目标沿 odom +X 的速度，单位 m/s",
            ),
            DeclareLaunchArgument(
                "target_world_yaw",
                default_value="0.0",
                description="没有 ArUco 航向测量时的目标航向回退值，单位 rad",
            ),
            DeclareLaunchArgument(
                "mounting_yaw_offset",
                default_value="0.0",
                description=(
                    "二维码朝向与目标车前方之间的安装偏角；若测得航向"
                    "整体反了 pi，请设为 3.14159265359"
                ),
            ),
            DeclareLaunchArgument(
                "prefer_velocity_heading",
                default_value="true",
                description=(
                    "目标运动时优先使用滤波后的运动方向作为目标航向"
                ),
            ),
            DeclareLaunchArgument(
                "live_target_update",
                default_value="true",
                description="是否用后续二维码运动实时更新 odom 目标位置",
            ),
            DeclareLaunchArgument(
                "camera_mount_x",
                default_value="0.35",
                description="相机光心在 ego_base 中的前向安装偏移，单位 m",
            ),
            DeclareLaunchArgument(
                "camera_mount_y",
                default_value="0.0",
                description="相机光心在 ego_base 中的横向安装偏移，单位 m",
            ),
            DeclareLaunchArgument(
                "camera_mount_z",
                default_value="0.20",
                description="相机光心在 ego_base 中的高度，单位 m",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="是否启动 RViz",
            ),
            camera_and_detector,
            qr_adapter,
            target_bridge,
            ego_simulator,
            camera_optical_tf,
            planner,
            rviz,
        ]
    )
