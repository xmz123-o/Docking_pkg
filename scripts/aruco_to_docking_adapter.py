#!/usr/bin/python3
"""将 ArUco 标记位姿转换为 Docking_pkg 可用的二维码位置消息。"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64


class ArucoToDockingAdapter(Node):
    def __init__(self):
        super().__init__('aruco_to_docking_adapter')

        self.input_topic = self.declare_parameter(
            'input_topic', '/aruco/pose').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/qr/tag_position').value
        self.yaw_output_topic = self.declare_parameter(
            'yaw_output_topic', '/qr/tag_yaw_camera').value
        self.mounting_yaw_offset = self.declare_parameter(
            'mounting_yaw_offset', 0.0).value
        self.output_frame = self.declare_parameter(
            'output_frame', 'camera_link').value
        self.stale_timeout = self.declare_parameter(
            'stale_timeout', 0.5).value
        self.min_range = self.declare_parameter(
            'min_range', 0.05).value
        self.max_range = self.declare_parameter(
            'max_range', 50.0).value
        self.axis_mode = self.declare_parameter(
            'axis_mode', 'optical_to_camera_link').value

        if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0.0:
            raise ValueError('stale_timeout 必须是有限正数')
        if not math.isfinite(self.mounting_yaw_offset):
            raise ValueError('mounting_yaw_offset 必须是有限数')
        if not math.isfinite(self.min_range) or not math.isfinite(self.max_range):
            raise ValueError('min_range/max_range 必须是有限数')
        if self.min_range < 0.0 or self.max_range <= self.min_range:
            raise ValueError('要求 0 <= min_range < max_range')
        if self.axis_mode not in ('optical_to_camera_link', 'passthrough'):
            raise ValueError(
                'axis_mode 必须是 optical_to_camera_link 或 passthrough')

        self.publisher = self.create_publisher(
            PointStamped, self.output_topic, 10)
        self.yaw_publisher = self.create_publisher(
            Float64, self.yaw_output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, self.input_topic, self.on_pose, 10)

        self.get_logger().info(
            f'适配器启动: {self.input_topic} -> {self.output_topic}, '
            f'output_frame={self.output_frame}, axis_mode={self.axis_mode}')

    def on_pose(self, msg):
        age = (
            self.get_clock().now() - Time.from_msg(msg.header.stamp)
        ).nanoseconds / 1e9
        if age > self.stale_timeout:
            self.get_logger().warning(
                f'丢弃过期位姿: age={age:.3f}s',
                throttle_duration_sec=2.0)
            return

        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.get_logger().warning(
                '丢弃非有限位姿', throttle_duration_sec=2.0)
            return

        if self.axis_mode == 'optical_to_camera_link':
            # 相机光学系: X 右、Y 下、Z 前
            # camera_link: X 前、Y 左、Z 上
            x = p.z
            y = -p.x
            z = -p.y
        else:
            x = p.x
            y = p.y
            z = p.z

        distance = math.sqrt(x * x + y * y + z * z)
        if distance < self.min_range or distance > self.max_range:
            self.get_logger().warning(
                f'丢弃超范围目标: range={distance:.3f}m',
                throttle_duration_sec=2.0)
            return

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.point.x = x
        out.point.y = y
        out.point.z = z
        self.publisher.publish(out)

        # 标记平面的 +Z 法向指向目标车后方，目标车前方取其反方向。
        q = msg.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            return
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        normal_optical = (
            2.0 * (qx * qz + qw * qy),
            2.0 * (qy * qz - qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        if self.axis_mode == 'optical_to_camera_link':
            nx = normal_optical[2]
            ny = -normal_optical[0]
        else:
            nx = normal_optical[0]
            ny = normal_optical[1]
        if math.hypot(nx, ny) < 1e-6:
            return
        target_yaw_camera = (
            math.atan2(-ny, -nx) + self.mounting_yaw_offset)
        target_yaw_camera = math.atan2(
            math.sin(target_yaw_camera), math.cos(target_yaw_camera))
        self.yaw_publisher.publish(Float64(data=target_yaw_camera))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoToDockingAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
