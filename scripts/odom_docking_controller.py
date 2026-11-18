#!/usr/bin/env python3
"""Fixed-frame docking controller for the real-QR/simulated-ego hybrid."""

import math

from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def build_hermite_path(
    start,
    goal,
    sample_count=70,
    tangent_scale=0.20,
):
    """生成起点切向为自车航向、终点切向为目标航向的三次 Bezier 路径。"""

    chord = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if chord < 1e-6:
        return [(goal[0], goal[1], goal[2], 0.0)]

    control_length = clamp(tangent_scale * chord, 0.08, 1.20)
    p0 = (start[0], start[1])
    p1 = (
        p0[0] + control_length * math.cos(start[2]),
        p0[1] + control_length * math.sin(start[2]),
    )
    p3 = (goal[0], goal[1])
    p2 = (
        p3[0] - control_length * math.cos(goal[2]),
        p3[1] - control_length * math.sin(goal[2]),
    )

    points = []
    total_s = 0.0
    previous_x = p0[0]
    previous_y = p0[1]
    samples = max(20, int(sample_count))
    for index in range(samples + 1):
        u = index / samples
        one_minus_u = 1.0 - u
        x = (
            one_minus_u**3 * p0[0]
            + 3.0 * one_minus_u**2 * u * p1[0]
            + 3.0 * one_minus_u * u**2 * p2[0]
            + u**3 * p3[0]
        )
        y = (
            one_minus_u**3 * p0[1]
            + 3.0 * one_minus_u**2 * u * p1[1]
            + 3.0 * one_minus_u * u**2 * p2[1]
            + u**3 * p3[1]
        )
        if index:
            total_s += math.hypot(x - previous_x, y - previous_y)
        points.append((x, y, total_s))
        previous_x = x
        previous_y = y

    path = []
    for index, (x, y, s) in enumerate(points):
        if index == 0:
            yaw = start[2]
        elif index == len(points) - 1:
            yaw = goal[2]
        else:
            left = points[index - 1]
            right = points[index + 1]
            yaw = math.atan2(right[1] - left[1], right[0] - left[0])
        path.append((x, y, wrap_angle(yaw), s))
    return path


def path_point_at_distance(path, distance):
    if not path:
        return (0.0, 0.0, 0.0, 0.0)
    if distance <= path[0][3]:
        return path[0]
    if distance >= path[-1][3]:
        return path[-1]
    for index in range(1, len(path)):
        right = path[index]
        if distance <= right[3]:
            left = path[index - 1]
            span = right[3] - left[3]
            ratio = (distance - left[3]) / span if span > 1e-9 else 0.0
            return (
                left[0] + ratio * (right[0] - left[0]),
                left[1] + ratio * (right[1] - left[1]),
                wrap_angle(left[2] + ratio * wrap_angle(right[2] - left[2])),
                distance,
            )
    return path[-1]


class OdomDockingController(Node):
    APPROACH = 'APPROACH'
    FINAL_DOCK = 'FINAL_DOCK'
    DOCKED = 'DOCKED'
    WAITING = 'WAITING_FOR_TARGET'

    def __init__(self):
        super().__init__('odom_docking_controller')

        self.desired_distance = float(
            self.declare_parameter('desired_center_distance', 0.52).value)
        self.align_heading_tolerance = float(
            self.declare_parameter('align_heading_tolerance', 0.06).value)
        self.align_lateral_tolerance = float(
            self.declare_parameter('align_lateral_tolerance', 0.04).value)
        self.dock_distance_tolerance = float(
            self.declare_parameter('dock_distance_tolerance', 0.025).value)
        self.dock_lateral_tolerance = float(
            self.declare_parameter('dock_lateral_tolerance', 0.03).value)
        self.final_press_speed = float(
            self.declare_parameter('final_press_speed', 0.04).value)
        self.release_distance = float(
            self.declare_parameter('release_distance', 0.06).value)
        self.release_lateral = float(
            self.declare_parameter('release_lateral', 0.06).value)

        self.approach_speed = float(
            self.declare_parameter('approach_speed', 0.40).value)
        self.final_speed = float(
            self.declare_parameter('final_speed', 0.20).value)
        self.target_motion_threshold = float(
            self.declare_parameter('target_motion_threshold', 0.05).value)
        self.max_accel = float(
            self.declare_parameter('max_accel', 0.60).value)
        self.max_decel = float(
            self.declare_parameter('max_decel', 0.80).value)
        self.heading_gain = float(
            self.declare_parameter('heading_gain', 1.20).value)
        self.bearing_gain = float(
            self.declare_parameter('bearing_gain', 0.80).value)
        self.yaw_rate_gain = float(
            self.declare_parameter('yaw_rate_gain', 0.30).value)
        self.max_angular = float(
            self.declare_parameter('max_angular', 0.60).value)
        self.axis_correction_gain = float(
            self.declare_parameter('axis_correction_gain', 1.20).value)
        self.path_tangent_scale = float(
            self.declare_parameter('path_tangent_scale', 0.20).value)
        self.lookahead_min = float(
            self.declare_parameter('lookahead_min', 0.32).value)
        self.lookahead_max = float(
            self.declare_parameter('lookahead_max', 0.65).value)
        self.heading_preview_gain = float(
            self.declare_parameter('heading_preview_gain', 0.55).value)
        self.terminal_pose_distance = float(
            self.declare_parameter('terminal_pose_distance', 0.15).value)

        self.wheel_radius = float(
            self.declare_parameter('wheel_radius', 0.254).value)
        self.wheel_track = float(
            self.declare_parameter('wheel_track', 0.60).value)
        self.max_wheel_rad_s = float(
            self.declare_parameter('max_wheel_rad_s', 8.0).value)
        self.control_period = float(
            self.declare_parameter('control_period', 0.05).value)

        self.target = None
        self.target_twist = (0.0, 0.0)
        self.ego = None
        self.ego_yaw_rate = 0.0
        self.phase = self.WAITING
        self.previous_speed = 0.0
        self.dock_hold_count = 0
        self.has_docked = False
        self.current_path = []
        self.last_time = self.get_clock().now()

        self.path_pub = self.create_publisher(Path, '/planning/path', 5)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/planning/markers', 5)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_cmd', 5)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.status_pub = self.create_publisher(
            String, '/docking/status', 5)
        self.docked_pub = self.create_publisher(
            Bool, '/position_only/docked', 5)

        self.create_subscription(
            PoseStamped, '/hybrid_target/pose', self.on_target, 10)
        self.create_subscription(
            TwistStamped, '/hybrid_target/twist', self.on_target_twist, 10)
        self.create_subscription(
            PoseStamped, '/ego/pose', self.on_ego, 20)
        self.create_subscription(
            TwistStamped, '/ego/twist', self.on_ego_twist, 20)

        self.timer = self.create_timer(
            max(0.01, self.control_period), self.on_timer)
        self.get_logger().info(
            'Odom docking controller started: '
            f'desired_distance={self.desired_distance:.3f}')

    def on_target(self, msg):
        p = msg.pose.position
        self.target = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(msg.pose.orientation),
        )

    def on_target_twist(self, msg):
        self.target_twist = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
        )

    def on_ego(self, msg):
        p = msg.pose.position
        self.ego = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(msg.pose.orientation),
        )

    def on_ego_twist(self, msg):
        self.ego_yaw_rate = float(msg.twist.angular.z)

    def desired_pose(self):
        distance = self.desired_distance
        tx, ty, tyaw = self.target
        return (
            tx - distance * math.cos(tyaw),
            ty - distance * math.sin(tyaw),
            tyaw,
        )

    @staticmethod
    def pose_error(ego, goal):
        dx = goal[0] - ego[0]
        dy = goal[1] - ego[1]
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if distance > 1e-6 else 0.0
        return distance, bearing, wrap_angle(goal[2] - ego[2])

    def axis_errors(self):
        tx, ty, tyaw = self.target
        dx = tx - self.ego[0]
        dy = ty - self.ego[1]
        along = math.cos(tyaw) * dx + math.sin(tyaw) * dy
        lateral = -math.sin(tyaw) * dx + math.cos(tyaw) * dy
        return along - self.desired_distance, lateral

    def limited_speed(self, desired, dt):
        upper = self.previous_speed + self.max_accel * dt
        lower = self.previous_speed - self.max_decel * dt
        return clamp(desired, max(0.0, lower), upper)

    def follow_path(
        self,
        path,
        max_speed,
        feedforward_speed,
        along_error,
        lateral_error,
        dt,
    ):
        if not path:
            return 0.0, 0.0
        lookahead = clamp(
            0.28 + 0.45 * abs(self.previous_speed),
            self.lookahead_min,
            self.lookahead_max,
        )
        carrot = path_point_at_distance(
            path, min(path[-1][3], lookahead))
        bearing = math.atan2(
            carrot[1] - self.ego[1], carrot[0] - self.ego[0])
        bearing_error = wrap_angle(bearing - self.ego[2])
        path_yaw_error = wrap_angle(carrot[2] - self.ego[2])
        remaining = path[-1][3]
        goal_yaw_error = wrap_angle(path[-1][2] - self.ego[2])

        # 从远处就开始混入目标轴线方向，而不是到最后几十厘米才回正。
        axis_denominator = max(0.22, self.desired_distance + along_error)
        lateral_for_heading = math.copysign(
            max(
                0.0,
                abs(lateral_error) - self.dock_lateral_tolerance,
            ),
            lateral_error,
        )
        axis_heading = wrap_angle(
            path[-1][2]
            + math.atan2(
                self.axis_correction_gain * lateral_for_heading,
                axis_denominator,
            )
        )
        axis_weight = clamp(1.0 - remaining / 1.20, 0.35, 0.95)
        desired_heading = wrap_angle(
            carrot[2] + axis_weight * wrap_angle(
                axis_heading - carrot[2]))
        desired_heading_error = wrap_angle(
            desired_heading - self.ego[2])

        desired_speed = clamp(
            feedforward_speed + 0.70 * along_error,
            0.0,
            max_speed,
        )
        if abs(lateral_error) > self.dock_lateral_tolerance:
            desired_speed = max(desired_speed, self.final_press_speed)
        if abs(bearing_error) > 0.90:
            desired_speed = min(desired_speed, 0.03)
        elif abs(bearing_error) > 0.60:
            desired_speed = min(desired_speed, 0.10)
        if abs(desired_heading_error) > 0.75:
            desired_speed = min(desired_speed, 0.04)
        elif abs(desired_heading_error) > 0.40:
            desired_speed = min(desired_speed, 0.14)
        angular = (
            self.heading_preview_gain * desired_heading_error +
            self.bearing_gain * 0.35 * bearing_error +
            0.20 * path_yaw_error +
            0.18 * goal_yaw_error -
            self.yaw_rate_gain * self.ego_yaw_rate
        )
        linear = self.limited_speed(desired_speed, dt)
        return linear, angular

    def terminal_pose_control(
        self,
        along_error,
        lateral_error,
        speed_limit,
        feedforward_speed,
        dt,
    ):
        """在路径终点用位姿反馈收尾，避免预瞄点退回自车后方位姿失稳。"""

        lateral_for_heading = math.copysign(
            max(
                0.0,
                abs(lateral_error) - self.dock_lateral_tolerance,
            ),
            lateral_error,
        )
        desired_heading = wrap_angle(
            self.target[2]
            + math.atan2(
                self.axis_correction_gain * lateral_for_heading,
                max(0.18, self.desired_distance + along_error),
            )
        )
        desired_heading_error = wrap_angle(
            desired_heading - self.ego[2])
        angular = (
            self.heading_gain * desired_heading_error
            - self.yaw_rate_gain * self.ego_yaw_rate
        )

        lateral_excess = max(
            0.0, abs(lateral_error) - self.dock_lateral_tolerance)
        desired_speed = clamp(
            feedforward_speed
            + 0.45 * along_error
            + 0.30 * lateral_excess,
            0.0,
            speed_limit if feedforward_speed > 0.0 else self.final_press_speed,
        )
        return self.limited_speed(desired_speed, dt), angular

    def on_timer(self):
        now = self.get_clock().now()
        dt = clamp((now - self.last_time).nanoseconds * 1e-9, 1e-3, 0.20)
        self.last_time = now

        if self.target is None or self.ego is None:
            self.publish_command(0.0, 0.0)
            self.publish_status(0.0, 0.0, 0.0)
            return

        along_error, lateral = self.axis_errors()
        center_distance = self.desired_distance + along_error
        heading_error = wrap_angle(self.target[2] - self.ego[2])
        target_vx, target_vy = self.target_twist
        target_speed = math.hypot(target_vx, target_vy)
        target_along_speed = (
            target_vx * math.cos(self.target[2]) +
            target_vy * math.sin(self.target[2]))
        target_moving = target_speed > self.target_motion_threshold

        if self.phase == self.DOCKED:
            released = (
                abs(along_error) > self.release_distance or
                abs(lateral) > self.release_lateral or
                abs(heading_error) > self.align_heading_tolerance
            )
            if target_moving:
                # 目标仍在运动时不在接触点停车，持续匹配目标速度。
                self.phase = self.FINAL_DOCK
                self.dock_hold_count = 0
            elif released:
                self.phase = (
                    self.FINAL_DOCK
                    if center_distance <= self.desired_distance + 0.20
                    else self.APPROACH
                )
                self.dock_hold_count = 0
            else:
                self.publish_command(0.0, 0.0)
                self.publish_path_and_markers(now)
                self.publish_status(
                    center_distance, lateral, heading_error)
                return

        final_goal = self.desired_pose()
        final_distance, _, _ = self.pose_error(self.ego, final_goal)

        if self.phase == self.WAITING:
            self.phase = self.APPROACH
        if self.phase == self.APPROACH and final_distance <= 0.35:
            self.phase = self.FINAL_DOCK
        if (
            self.phase == self.FINAL_DOCK
            and final_distance > 0.70
            and not self.has_docked
        ):
            self.phase = self.APPROACH

        if self.phase == self.APPROACH:
            speed_limit = self.approach_speed
        else:
            speed_limit = (
                self.approach_speed
                if self.has_docked or target_moving
                else self.final_speed
            )
        self.current_path = build_hermite_path(
            self.ego,
            final_goal,
            tangent_scale=self.path_tangent_scale,
        )
        if final_distance <= self.terminal_pose_distance:
            # 最终接触段只做很小的位姿收尾。
            linear, angular = self.terminal_pose_control(
                along_error,
                lateral,
                speed_limit,
                target_along_speed if target_moving else 0.0,
                dt,
            )
        else:
            linear, angular = self.follow_path(
                self.current_path,
                speed_limit,
                target_along_speed if target_moving else 0.0,
                along_error,
                lateral,
                dt,
            )

        # 目标运动时至少匹配其纵向速度；接触后不会再次停车追赶。
        if target_moving:
            linear = max(
                linear,
                min(target_along_speed, speed_limit),
            )

        angular = clamp(
            angular, -self.max_angular, self.max_angular)
        self.previous_speed = linear
        self.publish_command(linear, angular)
        self.publish_path_and_markers(now)
        self.publish_status(center_distance, lateral, heading_error)

        docked = (
            abs(along_error) <= self.dock_distance_tolerance and
            abs(lateral) <= self.dock_lateral_tolerance and
            abs(heading_error) <= self.align_heading_tolerance
        )
        self.dock_hold_count = self.dock_hold_count + 1 if docked else 0
        if self.dock_hold_count >= 3:
            self.has_docked = True
            if target_moving:
                # 保持对接状态，但继续以目标速度跟随，不产生停止-追赶振荡。
                self.phase = self.FINAL_DOCK
            else:
                self.phase = self.DOCKED
                self.publish_command(0.0, 0.0)
            self.publish_status(center_distance, lateral, heading_error)

    def publish_command(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

        left_linear = linear - angular * self.wheel_track / 2.0
        right_linear = linear + angular * self.wheel_track / 2.0
        left = clamp(
            left_linear / self.wheel_radius,
            -self.max_wheel_rad_s,
            self.max_wheel_rad_s,
        )
        right = clamp(
            right_linear / self.wheel_radius,
            -self.max_wheel_rad_s,
            self.max_wheel_rad_s,
        )
        self.wheel_pub.publish(Float64MultiArray(data=[left, right]))

    def publish_path_and_markers(self, stamp):
        if self.current_path:
            path_points = self.current_path
        else:
            goal = self.desired_pose()
            path_points = [
                (
                    self.ego[0] + (i / 30.0) * (goal[0] - self.ego[0]),
                    self.ego[1] + (i / 30.0) * (goal[1] - self.ego[1]),
                    goal[2],
                    i / 30.0,
                )
                for i in range(31)
            ]
        path = Path()
        path.header.stamp = stamp.to_msg()
        path.header.frame_id = 'odom'
        for point in path_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.z = math.sin(point[2] / 2.0)
            pose.pose.orientation.w = math.cos(point[2] / 2.0)
            path.poses.append(pose)
        self.path_pub.publish(path)

        marker = Marker()
        marker.header = path.header
        marker.ns = 'trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(
                x=point[0],
                y=point[1],
                z=0.02,
            )
            for point in path_points
        ]
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_status(self, distance, lateral, heading):
        display_phase = self.phase
        if self.has_docked and self.phase == self.FINAL_DOCK:
            display_phase = 'DOCKED_FOLLOW'
        target_yaw = self.target[2] if self.target is not None else 0.0
        ego_yaw = self.ego[2] if self.ego is not None else 0.0
        text = (
            f'{display_phase} dist={distance:.3f} '
            f'lat_err={lateral:.3f} yaw_err={heading:.3f} '
            f'target_yaw={target_yaw:.3f} '
            f'ego_yaw={ego_yaw:.3f} '
            f'wheel_cmd={self.previous_speed:.3f}')
        self.status_pub.publish(String(data=text))
        self.docked_pub.publish(Bool(data=self.has_docked))
        self.get_logger().info(text, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdomDockingController()
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
