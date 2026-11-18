#!/usr/bin/env python3
"""Bridge a real ArUco target observation into the simulated ego closed loop.

The first valid ArUco observation anchors the target in odom.  The bridge then
simulates what the camera would observe while the virtual ego moves.  This is
required when the camera is real but the ego pose/wheel feedback is simulated.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from visualization_msgs.msg import Marker, MarkerArray


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate(x, y, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return c * x - s * y, s * x + c * y


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def measured_heading_to_world(
    target_yaw_camera,
    anchor_ego_yaw,
    camera_yaw,
    tag_yaw,
):
    """把 camera_link 中的目标前向航向转换到锚定时的 odom。"""

    return wrap_angle(
        anchor_ego_yaw + camera_yaw + target_yaw_camera - tag_yaw)


def measured_tag_to_world(
    point_camera,
    ego_x,
    ego_y,
    ego_yaw,
    camera_x,
    camera_y,
    camera_yaw,
):
    """只把二维码中心转换到 odom，不依赖目标航向。"""

    tag_ego_x, tag_ego_y = rotate(
        point_camera[0], point_camera[1], camera_yaw)
    tag_ego_x += camera_x
    tag_ego_y += camera_y
    world_dx, world_dy = rotate(tag_ego_x, tag_ego_y, ego_yaw)
    return ego_x + world_dx, ego_y + world_dy


def measured_tag_to_target_world(
    point_camera,
    ego_x,
    ego_y,
    ego_yaw,
    camera_x,
    camera_y,
    camera_yaw,
    tag_x,
    tag_y,
    tag_yaw,
    target_world_yaw,
):
    """Convert one camera_link ArUco point to an odom target-center pose."""

    tag_ego_x, tag_ego_y = rotate(point_camera[0], point_camera[1], camera_yaw)
    tag_ego_x += camera_x
    tag_ego_y += camera_y

    target_body_yaw = wrap_angle(target_world_yaw - ego_yaw)
    offset_x, offset_y = rotate(tag_x, tag_y, tag_yaw)
    offset_x, offset_y = rotate(offset_x, offset_y, target_body_yaw)
    target_ego_x = tag_ego_x - offset_x
    target_ego_y = tag_ego_y - offset_y

    world_dx, world_dy = rotate(target_ego_x, target_ego_y, ego_yaw)
    return ego_x + world_dx, ego_y + world_dy


def target_world_to_tag_camera(
    target_world,
    ego_x,
    ego_y,
    ego_yaw,
    camera_x,
    camera_y,
    camera_yaw,
    tag_x,
    tag_y,
    tag_yaw,
    target_world_yaw,
):
    """Generate the camera_link point that the simulated camera would measure."""

    relative_dx = target_world[0] - ego_x
    relative_dy = target_world[1] - ego_y
    target_ego_x, target_ego_y = rotate(
        relative_dx, relative_dy, -ego_yaw
    )

    target_body_yaw = wrap_angle(target_world_yaw - ego_yaw)
    offset_x, offset_y = rotate(tag_x, tag_y, tag_yaw)
    offset_x, offset_y = rotate(offset_x, offset_y, target_body_yaw)
    tag_ego_x = target_ego_x + offset_x
    tag_ego_y = target_ego_y + offset_y

    camera_dx = tag_ego_x - camera_x
    camera_dy = tag_ego_y - camera_y
    return rotate(camera_dx, camera_dy, -camera_yaw)


class HybridTargetBridge(Node):
    def __init__(self):
        super().__init__("hybrid_target_bridge")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/qr/tag_measurement").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/qr/tag_position").value
        )
        self.ego_pose_topic = str(
            self.declare_parameter("ego_pose_topic", "/ego/pose").value
        )
        self.reset_topic = str(
            self.declare_parameter("reset_topic", "/qr/hybrid/reanchor").value
        )
        self.output_frame = str(
            self.declare_parameter("output_frame", "camera_link").value
        )
        self.marker_topic = str(
            self.declare_parameter(
                "marker_topic", "/hybrid_target/markers").value
        )
        self.pose_topic = str(
            self.declare_parameter(
                "pose_topic", "/hybrid_target/pose").value
        )
        self.twist_topic = str(
            self.declare_parameter(
                "twist_topic", "/hybrid_target/twist").value
        )
        self.target_length = float(
            self.declare_parameter("target_length", 0.52).value
        )
        self.target_width = float(
            self.declare_parameter("target_width", 0.45).value
        )
        self.target_height = float(
            self.declare_parameter("target_height", 0.40).value
        )
        self.tag_on_rear_face = bool(
            self.declare_parameter("tag_on_rear_face", True).value
        )

        self.camera_x = float(self.declare_parameter("camera_x", 0.35).value)
        self.camera_y = float(self.declare_parameter("camera_y", 0.0).value)
        self.camera_yaw = float(self.declare_parameter("camera_yaw", 0.0).value)
        configured_tag_x = float(self.declare_parameter("tag_x", -0.26).value)
        self.tag_x = (
            -0.5 * self.target_length
            if self.tag_on_rear_face
            else configured_tag_x
        )
        self.tag_y = float(self.declare_parameter("tag_y", 0.0).value)
        self.tag_yaw = float(self.declare_parameter("tag_yaw", 0.0).value)

        self.target_world_yaw = float(
            self.declare_parameter("target_world_yaw", 0.0).value
        )
        self.yaw_input_topic = str(
            self.declare_parameter(
                "yaw_input_topic", "/qr/tag_yaw_camera").value
        )
        self.use_measured_target_heading = bool(
            self.declare_parameter(
                "use_measured_target_heading", True).value
        )
        self.target_heading_filter_alpha = float(
            self.declare_parameter(
                "target_heading_filter_alpha", 0.25).value
        )
        self.heading_timeout = float(
            self.declare_parameter("heading_timeout", 1.0).value
        )
        self.heading_wait_timeout = float(
            self.declare_parameter("heading_wait_timeout", 0.25).value
        )
        self.heading_jump_tolerance = float(
            self.declare_parameter(
                "heading_jump_tolerance", 0.70).value
        )
        self.prefer_velocity_heading = bool(
            self.declare_parameter("prefer_velocity_heading", True).value
        )
        self.velocity_heading_min_speed = float(
            self.declare_parameter(
                "velocity_heading_min_speed", 0.06).value
        )
        self.velocity_heading_filter_alpha = float(
            self.declare_parameter(
                "velocity_heading_filter_alpha", 0.30).value
        )
        self.velocity_heading_required_samples = int(
            self.declare_parameter(
                "velocity_heading_required_samples", 5).value
        )
        self.velocity_heading_candidate_tolerance = float(
            self.declare_parameter(
                "velocity_heading_candidate_tolerance", 0.35).value
        )
        self.target_speed = float(
            self.declare_parameter("target_speed", 0.0).value
        )
        self.live_target_update = bool(
            self.declare_parameter("live_target_update", True).value
        )
        self.position_filter_alpha = float(
            self.declare_parameter("position_filter_alpha", 0.65).value
        )
        self.velocity_filter_alpha = float(
            self.declare_parameter("velocity_filter_alpha", 0.20).value
        )
        self.prediction_timeout = float(
            self.declare_parameter("prediction_timeout", 0.35).value
        )
        self.max_target_speed = float(
            self.declare_parameter("max_target_speed", 1.5).value
        )
        publish_hz = float(self.declare_parameter("publish_hz", 30.0).value)
        if not math.isfinite(publish_hz) or publish_hz <= 0.0:
            raise ValueError("publish_hz must be finite and positive")
        if (
            not math.isfinite(self.target_heading_filter_alpha)
            or self.target_heading_filter_alpha <= 0.0
            or self.target_heading_filter_alpha > 1.0
        ):
            raise ValueError(
                "target_heading_filter_alpha must be in (0, 1]")
        if not math.isfinite(self.heading_timeout) or self.heading_timeout <= 0.0:
            raise ValueError("heading_timeout must be finite and positive")
        if (
            not math.isfinite(self.heading_wait_timeout)
            or self.heading_wait_timeout < 0.0
        ):
            raise ValueError("heading_wait_timeout must be finite and nonnegative")
        if (
            not math.isfinite(self.heading_jump_tolerance)
            or self.heading_jump_tolerance <= 0.0
        ):
            raise ValueError(
                "heading_jump_tolerance must be finite and positive")
        if (
            not math.isfinite(self.velocity_heading_min_speed)
            or self.velocity_heading_min_speed < 0.0
        ):
            raise ValueError(
                "velocity_heading_min_speed must be finite and nonnegative")
        if (
            not math.isfinite(self.velocity_heading_filter_alpha)
            or self.velocity_heading_filter_alpha <= 0.0
            or self.velocity_heading_filter_alpha > 1.0
        ):
            raise ValueError(
                "velocity_heading_filter_alpha must be in (0, 1]")
        if self.velocity_heading_required_samples < 1:
            raise ValueError(
                "velocity_heading_required_samples must be positive")
        if (
            not math.isfinite(self.velocity_heading_candidate_tolerance)
            or self.velocity_heading_candidate_tolerance <= 0.0
        ):
            raise ValueError(
                "velocity_heading_candidate_tolerance must be positive")

        self.measurement = None
        self.measurement_yaw_camera = None
        self.ego_pose = None
        self.anchored = False
        self.anchor_ego_pose = None
        self.target_world = [0.0, 0.0]
        self.target_velocity_world = [0.0, 0.0]
        self.last_measurement_time = None
        self.first_measurement_time = None
        self.last_yaw_time = None
        self.measured_heading_initialized = False
        self.heading_jump_candidate = None
        self.heading_jump_count = 0
        self.velocity_heading_world = None
        self.velocity_heading_candidate = None
        self.velocity_heading_candidate_count = 0
        self.last_tag_world = None
        self.last_publish_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            PointStamped, self.output_topic, 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, 10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped, self.twist_topic, 10
        )
        self.create_subscription(PointStamped, self.input_topic, self.on_measurement, 10)
        self.create_subscription(Float64, self.yaw_input_topic, self.on_target_yaw, 10)
        self.create_subscription(PoseStamped, self.ego_pose_topic, self.on_ego_pose, 20)
        self.create_subscription(Bool, self.reset_topic, self.on_reset, 5)

        self.timer = self.create_timer(1.0 / publish_hz, self.on_timer)
        self.get_logger().info(
            "混合闭环桥接已启动：第一条有效二维码将目标锁定到 odom，"
            "随后根据虚拟自车运动生成新的相对观测。"
        )
        self.get_logger().info(
            f"目标二维码安装：rear_face={self.tag_on_rear_face}, "
            f"tag_x={self.tag_x:.3f} m"
        )
        self.get_logger().info(
            "目标航向来源："
            + (
                f"ArUco {self.yaw_input_topic}；"
                "目标运动时优先使用滤波后的运动方向"
                if self.use_measured_target_heading
                else f"固定参数 yaw={self.target_world_yaw:.3f}"
            )
        )

    def on_reset(self, msg):
        if not msg.data:
            return
        self.anchored = False
        self.anchor_ego_pose = None
        self.measurement = None
        self.measurement_yaw_camera = None
        self.last_measurement_time = None
        self.first_measurement_time = None
        self.last_yaw_time = None
        self.measured_heading_initialized = False
        self.heading_jump_candidate = None
        self.heading_jump_count = 0
        self.velocity_heading_world = None
        self.velocity_heading_candidate = None
        self.velocity_heading_candidate_count = 0
        self.last_tag_world = None
        self.target_world_yaw = float(
            self.get_parameter("target_world_yaw").value)
        self.target_velocity_world = [0.0, 0.0]
        self.get_logger().info("收到重新锁定请求，等待下一次有效二维码")

    def on_measurement(self, msg):
        p = msg.point
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.measurement = (p.x, p.y, p.z)
            if self.first_measurement_time is None:
                self.first_measurement_time = self.get_clock().now()
            if self.anchored:
                if self.live_target_update:
                    self.update_target_heading_from_measurement()
                    self.update_target_from_measurement()
            else:
                self.try_anchor()

    def on_target_yaw(self, msg):
        if not math.isfinite(msg.data):
            return
        self.measurement_yaw_camera = wrap_angle(float(msg.data))
        self.last_yaw_time = self.get_clock().now()
        if self.anchored:
            self.update_target_heading_from_measurement()

    def on_ego_pose(self, msg):
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y)):
            return
        self.ego_pose = (p.x, p.y, yaw_from_quaternion(msg.pose.orientation))
        self.try_anchor()

    def try_anchor(self):
        if self.anchored or self.measurement is None or self.ego_pose is None:
            return
        if self.use_measured_target_heading:
            now = self.get_clock().now()
            yaw_ready = False
            if self.measurement_yaw_camera is not None and self.last_yaw_time is not None:
                yaw_age = (now - self.last_yaw_time).nanoseconds * 1e-9
                yaw_ready = yaw_age <= self.heading_timeout
            if not yaw_ready and self.first_measurement_time is not None:
                measurement_age = (
                    now - self.first_measurement_time).nanoseconds * 1e-9
                if measurement_age < self.heading_wait_timeout:
                    return

        self.anchor_ego_pose = self.ego_pose
        self.update_target_heading_from_measurement()
        self.update_target_from_measurement()
        self.anchored = True
        self.get_logger().info(
            f"目标已锁定到 odom：x={self.target_world[0]:.3f}, "
            f"y={self.target_world[1]:.3f}, yaw={self.target_world_yaw:.3f}"
        )

    def update_target_heading_from_measurement(self):
        """用标记后壁法向更新目标前向，拒绝约 180 度的平面歧义翻转。"""

        if (
            not self.use_measured_target_heading
            or self.measurement_yaw_camera is None
            or self.anchor_ego_pose is None
            or self.last_yaw_time is None
        ):
            return
        if (
            self.prefer_velocity_heading
            and self.velocity_heading_world is not None
        ):
            # 目标停住后保持最后一次可信运动方向，不再被近距离标记噪声拉偏。
            self.target_world_yaw = self.velocity_heading_world
            return

        now = self.get_clock().now()
        age = (now - self.last_yaw_time).nanoseconds * 1e-9
        if age > self.heading_timeout:
            return

        _, _, anchor_ego_yaw = self.anchor_ego_pose
        candidate = measured_heading_to_world(
            self.measurement_yaw_camera,
            anchor_ego_yaw,
            self.camera_yaw,
            self.tag_yaw,
        )

        if not self.measured_heading_initialized:
            self.target_world_yaw = candidate
            self.measured_heading_initialized = True
            self.heading_jump_candidate = None
            self.heading_jump_count = 0
            self.get_logger().info(
                "首次采用 ArUco 测量航向："
                f"target_world_yaw={self.target_world_yaw:.3f} rad")
            return

        error = wrap_angle(candidate - self.target_world_yaw)
        # 单目平面标记偶尔会沿法向翻转，表现为约 pi 的跳变。
        if abs(error) > 0.5 * math.pi:
            flipped = wrap_angle(candidate + math.pi)
            flipped_error = wrap_angle(flipped - self.target_world_yaw)
            if abs(flipped_error) < abs(error):
                candidate = flipped
                error = flipped_error

        if abs(error) > self.heading_jump_tolerance:
            if (
                self.heading_jump_candidate is not None
                and abs(wrap_angle(candidate - self.heading_jump_candidate))
                < self.heading_jump_tolerance
            ):
                self.heading_jump_count += 1
            else:
                self.heading_jump_candidate = candidate
                self.heading_jump_count = 1
            if self.heading_jump_count < 3:
                return
            # 连续三次一致才接受真实的大角度变化。
            self.target_world_yaw = candidate
        else:
            alpha = self.target_heading_filter_alpha
            self.target_world_yaw = wrap_angle(
                self.target_world_yaw + alpha * error)
        self.heading_jump_candidate = None
        self.heading_jump_count = 0

    def update_target_from_measurement(self):
        """Update the odom target from a fixed-camera QR observation."""

        if self.measurement is None or self.anchor_ego_pose is None:
            return
        if self.use_measured_target_heading:
            self.update_target_heading_from_measurement()
        ego_x, ego_y, ego_yaw = self.anchor_ego_pose
        tag_world_x, tag_world_y = measured_tag_to_world(
            self.measurement,
            ego_x,
            ego_y,
            ego_yaw,
            self.camera_x,
            self.camera_y,
            self.camera_yaw,
        )
        target_x, target_y = measured_tag_to_target_world(
            self.measurement,
            ego_x,
            ego_y,
            ego_yaw,
            self.camera_x,
            self.camera_y,
            self.camera_yaw,
            self.tag_x,
            self.tag_y,
            self.tag_yaw,
            self.target_world_yaw,
        )
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        if self.last_tag_world is not None and self.last_measurement_time is not None:
            tag_dt = now_seconds - self.last_measurement_time
            if 1e-3 < tag_dt < 0.30:
                tag_vx = (tag_world_x - self.last_tag_world[0]) / tag_dt
                tag_vy = (tag_world_y - self.last_tag_world[1]) / tag_dt
                self.update_heading_from_velocity(tag_vx, tag_vy)
            else:
                self.velocity_heading_candidate = None
                self.velocity_heading_candidate_count = 0
        self.last_tag_world = [tag_world_x, tag_world_y]

        if self.last_measurement_time is None:
            self.target_world[0] = target_x
            self.target_world[1] = target_y
        else:
            dt = now_seconds - self.last_measurement_time
            if dt >= 0.30:
                # 检测间隔过长时不沿用旧速度，避免错误的远距离外推。
                self.target_world[0] = target_x
                self.target_world[1] = target_y
                self.target_velocity_world[0] = 0.0
                self.target_velocity_world[1] = 0.0
            elif dt > 1e-3:
                raw_vx = (target_x - self.target_world[0]) / dt
                raw_vy = (target_y - self.target_world[1]) / dt
                raw_speed = math.hypot(raw_vx, raw_vy)
                if raw_speed > self.max_target_speed:
                    scale = self.max_target_speed / raw_speed
                    raw_vx *= scale
                    raw_vy *= scale
                velocity_alpha = max(0.0, min(1.0, self.velocity_filter_alpha))
                self.target_velocity_world[0] = (
                    (1.0 - velocity_alpha) * self.target_velocity_world[0]
                    + velocity_alpha * raw_vx
                )
                self.target_velocity_world[1] = (
                    (1.0 - velocity_alpha) * self.target_velocity_world[1]
                    + velocity_alpha * raw_vy
                )
                position_alpha = max(0.0, min(1.0, self.position_filter_alpha))
                self.target_world[0] += position_alpha * (
                    target_x - self.target_world[0]
                )
                self.target_world[1] += position_alpha * (
                    target_y - self.target_world[1]
                )
        self.last_measurement_time = now_seconds

    def update_heading_from_velocity(self, raw_vx, raw_vy):
        """仅在二维码平移速度连续可信时使用运动方向。"""

        speed = math.hypot(raw_vx, raw_vy)
        if (
            not self.prefer_velocity_heading
            or speed < self.velocity_heading_min_speed
        ):
            self.velocity_heading_candidate = None
            self.velocity_heading_candidate_count = 0
            return

        raw_heading = math.atan2(raw_vy, raw_vx)
        if (
            self.velocity_heading_candidate is not None
            and abs(wrap_angle(
                raw_heading - self.velocity_heading_candidate
            )) <= self.velocity_heading_candidate_tolerance
        ):
            self.velocity_heading_candidate_count += 1
        else:
            self.velocity_heading_candidate = raw_heading
            self.velocity_heading_candidate_count = 1

        if (
            self.velocity_heading_candidate_count
            < self.velocity_heading_required_samples
        ):
            return

        if self.velocity_heading_world is None:
            self.velocity_heading_world = self.target_world_yaw
        alpha = self.velocity_heading_filter_alpha
        self.velocity_heading_world = wrap_angle(
            self.velocity_heading_world
            + alpha * wrap_angle(
                raw_heading - self.velocity_heading_world)
        )
        self.target_world_yaw = self.velocity_heading_world

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = now
        if not self.anchored or self.ego_pose is None:
            return

        if not self.live_target_update and abs(self.target_speed) > 1e-6:
            self.target_world[0] += (
                self.target_speed * math.cos(self.target_world_yaw) * dt
            )
            self.target_world[1] += (
                self.target_speed * math.sin(self.target_world_yaw) * dt
            )

        observed_target = list(self.target_world)
        if (
            self.live_target_update
            and self.last_measurement_time is not None
            and self.prediction_timeout > 0.0
        ):
            now_seconds = now.nanoseconds * 1e-9
            measurement_age = max(
                0.0, now_seconds - self.last_measurement_time
            )
            prediction_age = min(measurement_age, self.prediction_timeout)
            observed_target[0] += self.target_velocity_world[0] * prediction_age
            observed_target[1] += self.target_velocity_world[1] * prediction_age

        ego_x, ego_y, ego_yaw = self.ego_pose
        tag_x_camera, tag_y_camera = target_world_to_tag_camera(
            observed_target,
            ego_x,
            ego_y,
            ego_yaw,
            self.camera_x,
            self.camera_y,
            self.camera_yaw,
            self.tag_x,
            self.tag_y,
            self.tag_yaw,
            self.target_world_yaw,
        )

        output = PointStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = self.output_frame
        output.point.x = tag_x_camera
        output.point.y = tag_y_camera
        output.point.z = 0.0
        self.publisher.publish(output)
        self.publish_target_marker(now, observed_target)
        self.publish_target_pose(now, observed_target)

    def publish_target_marker(self, stamp, target_position):
        marker = Marker()
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = target_position[0]
        marker.pose.position.y = target_position[1]
        marker.pose.position.z = self.target_height / 2.0
        marker.pose.orientation.z = math.sin(self.target_world_yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.target_world_yaw / 2.0)
        marker.scale.x = self.target_length
        marker.scale.y = self.target_width
        marker.scale.z = self.target_height
        marker.color.r = 0.90
        marker.color.g = 0.25
        marker.color.b = 0.20
        marker.color.a = 0.90
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def publish_target_pose(self, stamp, target_position):
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = target_position[0]
        pose.pose.position.y = target_position[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.target_world_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.target_world_yaw / 2.0)
        self.pose_publisher.publish(pose)

        twist = TwistStamped()
        twist.header = pose.header
        if self.live_target_update:
            twist.twist.linear.x = self.target_velocity_world[0]
            twist.twist.linear.y = self.target_velocity_world[1]
        else:
            twist.twist.linear.x = (
                self.target_speed * math.cos(self.target_world_yaw))
            twist.twist.linear.y = (
                self.target_speed * math.sin(self.target_world_yaw))
        self.twist_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HybridTargetBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
