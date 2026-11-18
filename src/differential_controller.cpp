#include "docking_pkg/differential_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace docking
{

namespace
{

TrajectoryPoint interpolateTrajectory(const Trajectory & trajectory, double s)
{
  if (trajectory.empty()) {
    return TrajectoryPoint{};
  }
  if (s <= trajectory.front().s) {
    return trajectory.front();
  }
  if (s >= trajectory.back().s) {
    return trajectory.back();
  }
  for (size_t i = 1; i < trajectory.size(); ++i) {
    if (s <= trajectory[i].s) {
      const double s0 = trajectory[i - 1].s;
      const double s1 = trajectory[i].s;
      const double ratio = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
      TrajectoryPoint out;
      out.t = trajectory[i - 1].t + ratio * (trajectory[i].t - trajectory[i - 1].t);
      out.x = trajectory[i - 1].x + ratio * (trajectory[i].x - trajectory[i - 1].x);
      out.y = trajectory[i - 1].y + ratio * (trajectory[i].y - trajectory[i - 1].y);
      out.yaw = normalizeAngle(trajectory[i - 1].yaw +
        ratio * normalizeAngle(trajectory[i].yaw - trajectory[i - 1].yaw));
      out.v = trajectory[i - 1].v + ratio * (trajectory[i].v - trajectory[i - 1].v);
      out.kappa = trajectory[i - 1].kappa +
        ratio * (trajectory[i].kappa - trajectory[i - 1].kappa);
      out.omega = trajectory[i - 1].omega +
        ratio * (trajectory[i].omega - trajectory[i - 1].omega);
      out.s = s;
      return out;
    }
  }
  return trajectory.back();
}

PathPoint interpolatePathPoint(const Path & path, double s)
{
  if (path.empty()) {
    return PathPoint{};
  }
  if (s <= path.front().s) {
    return path.front();
  }
  if (s >= path.back().s) {
    return path.back();
  }
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) {
      const double s0 = path[i - 1].s;
      const double s1 = path[i].s;
      const double ratio = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
      PathPoint out;
      out.x = path[i - 1].x + ratio * (path[i].x - path[i - 1].x);
      out.y = path[i - 1].y + ratio * (path[i].y - path[i - 1].y);
      out.yaw = normalizeAngle(path[i - 1].yaw +
        ratio * normalizeAngle(path[i].yaw - path[i - 1].yaw));
      out.kappa = path[i - 1].kappa +
        ratio * (path[i].kappa - path[i - 1].kappa);
      out.s = s;
      return out;
    }
  }
  return path.back();
}

}  // 命名空间

DifferentialController::DifferentialController() : DifferentialController(Params{}) {}

DifferentialController::DifferentialController(const Params & params) : params_(params) {}

WheelCommand DifferentialController::compute(
  const Pose2D & ego_pose,
  const Trajectory & trajectory,
  double cruise_speed) const
{
  WheelCommand command;
  if (trajectory.empty()) {
    return command;
  }

  double nearest_s = trajectory.front().s;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto & point : trajectory) {
    const double d = squared(point.x - ego_pose.x) + squared(point.y - ego_pose.y);
    if (d < best_distance) {
      best_distance = d;
      nearest_s = point.s;
    }
  }

  const TrajectoryPoint nearest_point =
    interpolateTrajectory(trajectory, nearest_s);
  const double nominal_speed = cruise_speed > 0.0
    ? std::min(nearest_point.v, cruise_speed)
    : 0.0;
  const double lookahead_distance = clip(
    nominal_speed * params_.lookahead_time,
    params_.lookahead_distance,
    params_.max_lookahead_distance);
  const double target_s = std::min(
    trajectory.back().s, nearest_s + lookahead_distance);
  const TrajectoryPoint lookahead = interpolateTrajectory(trajectory, target_s);

  const double dx = lookahead.x - ego_pose.x;
  const double dy = lookahead.y - ego_pose.y;
  const double local_x = std::cos(ego_pose.yaw) * dx + std::sin(ego_pose.yaw) * dy;
  const double local_y = -std::sin(ego_pose.yaw) * dx + std::cos(ego_pose.yaw) * dy;
  const double lookahead_dist = std::hypot(local_x, local_y);

  double linear = lookahead.v;
  if (cruise_speed <= 0.0) {
    linear = 0.0;
  } else {
    linear = std::min(linear, cruise_speed);
  }

  double angular = 0.0;
  if (lookahead_dist > 1e-3) {
    const double curvature = 2.0 * local_y / (lookahead_dist * lookahead_dist);
    const double pursuit_omega = linear * curvature;
    const double heading_error = normalizeAngle(lookahead.yaw - ego_pose.yaw);
    angular = 0.5 * pursuit_omega +
      params_.path_feedforward_gain * lookahead.omega +
      params_.path_heading_gain * heading_error;
  }

  linear = clip(linear, -params_.max_linear, params_.max_linear);
  angular = clip(angular, -params_.max_angular, params_.max_angular);

  return fromBodyVelocity(linear, angular);
}

WheelCommand DifferentialController::fromTwist(
  double linear, double angular) const
{
  return fromBodyVelocity(linear, angular);
}

WheelCommand DifferentialController::fromBodyVelocity(
  double linear, double angular) const
{
  WheelCommand command;
  linear = clip(linear, -params_.max_linear, params_.max_linear);
  angular = clip(angular, -params_.max_angular, params_.max_angular);

  const double half_track = params_.wheel_track / 2.0;
  const double wheel_linear_left = linear - angular * half_track;
  const double wheel_linear_right = linear + angular * half_track;

  command.left_rad_s = clip(
    wheel_linear_left / params_.wheel_radius, -params_.max_wheel_rad_s, params_.max_wheel_rad_s);
  command.right_rad_s = clip(
    wheel_linear_right / params_.wheel_radius, -params_.max_wheel_rad_s, params_.max_wheel_rad_s);
  command.linear = linear;
  command.angular = angular;
  return command;
}

WheelCommand DifferentialController::computeBodyRelative(
  double ego_speed,
  const TargetState & target,
  double desired_center_distance) const
{
  WheelCommand command;
  if (!target.initialized) {
    return command;
  }

  const double dx = target.x;
  const double dy = target.y;
  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);

  const double along = tx * dx + ty * dy;
  const double lateral = -tx * dy + ty * dx;
  const double along_error = along - desired_center_distance;

  const double v_along = tx * target.vx + ty * target.vy;
  const double v_lateral = -tx * target.vy + ty * target.vx;

  const double desired_v_along = clip(
    -params_.body_k_along * along_error,
    -params_.body_max_closure,
    params_.body_max_closure);
  double linear = ego_speed +
    params_.body_k_along_vel * (v_along - desired_v_along);
  // 允许负速度用于对接过冲后的短距离后退修正。

  double angular =
    target.omega +
    params_.body_k_heading * normalizeAngle(target.yaw) +
    params_.body_k_lateral * lateral +
    params_.body_k_lateral_vel * v_lateral;

  return fromTwist(linear, angular);
}

WheelCommand DifferentialController::computeBodyRelativePath(
  double ego_speed,
  const TargetState & target,
  const Path & path,
  double desired_center_distance,
  double ego_yaw_rate) const
{
  WheelCommand command;
  if (!target.initialized) {
    return command;
  }

  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);
  const double along = tx * target.x + ty * target.y;
  const double along_error = along - desired_center_distance;
  const double v_along = tx * target.vx + ty * target.vy;

  // target.vx/vy 是相对速度；负的期望相对速度表示自车略快于目标。
  const double desired_v_along = clip(
    -params_.body_k_along * along_error,
    -params_.body_max_closure,
    params_.body_max_closure);
  double linear = ego_speed +
    params_.body_k_along_vel * (v_along - desired_v_along);
  // 允许负速度用于对接过冲后的短距离后退修正。

  const double docking_goal_x =
    target.x - desired_center_distance * std::cos(target.yaw);
  const double docking_goal_y =
    target.y - desired_center_distance * std::sin(target.yaw);
  const double docking_error = std::hypot(docking_goal_x, docking_goal_y);

  double angular = target.omega;
  if (!path.empty()) {
    const double lookahead_distance = clip(
      linear * params_.lookahead_time,
      params_.lookahead_distance,
      params_.max_lookahead_distance);
    const PathPoint carrot = interpolatePathPoint(
      path, std::min(path.back().s, lookahead_distance));
    const double carrot_distance = std::hypot(carrot.x, carrot.y);
    const double alpha = carrot_distance > 1e-3
      ? std::atan2(carrot.y, carrot.x)
      : 0.0;
    const double pursuit_curvature = carrot_distance > 1e-3
      ? 2.0 * std::sin(alpha) / std::max(0.05, carrot_distance)
      : 0.0;

    angular +=
      params_.path_heading_gain * alpha +
      params_.path_feedforward_gain * linear * pursuit_curvature;

    // 末端所剩对接距离很短时，五次样条端点切向约束会放大位置噪声，
    // 直接过渡到目标中心方位与目标航向控制。
    if (docking_error < params_.body_terminal_transition_distance &&
      std::hypot(target.x, target.y) > 0.05)
    {
      const double bearing = std::atan2(target.y, target.x);
      const double blend = clip(
        (params_.body_terminal_transition_distance - docking_error) /
        std::max(0.01, params_.body_terminal_transition_distance),
        0.0,
        1.0);
      const double terminal_angular =
        params_.body_terminal_heading_gain * target.yaw +
        params_.body_terminal_lateral_gain * bearing -
        params_.body_terminal_yaw_rate_gain * ego_yaw_rate;
      angular = (1.0 - blend) * angular +
        blend * terminal_angular;
    }
  } else {
    const double lateral = -ty * target.y + tx * target.x;
    const double v_lateral = -ty * target.vy + tx * target.vx;
    angular +=
      params_.body_k_heading * target.yaw +
      params_.body_k_lateral * lateral +
      params_.body_k_lateral_vel * v_lateral;
  }

  return fromTwist(linear, angular);
}

WheelCommand DifferentialController::computeBodyAlignment(
  double ego_speed,
  const TargetState & target,
  double desired_center_distance,
  double ego_yaw_rate,
  int * phase) const
{
  WheelCommand command;
  if (!target.initialized || phase == nullptr) {
    return command;
  }

  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);
  const double along = tx * target.x + ty * target.y;
  const double lateral = -ty * target.x + tx * target.y;
  const double heading_error = normalizeAngle(target.yaw);
  const double along_error = along - desired_center_distance;
  const double v_along = tx * target.vx + ty * target.vy;
  const double desired_v_along = clip(
    -params_.body_k_along * along_error,
    -params_.body_max_closure,
    params_.body_max_closure);
  double linear = ego_speed +
    params_.body_k_along_vel * (v_along - desired_v_along);

  const bool heading_aligned =
    std::abs(heading_error) <= params_.body_alignment_heading_tolerance;
  const double center_distance = std::hypot(target.x, target.y);

  const double standoff_distance =
    desired_center_distance + params_.body_alignment_standoff_offset;
  const double standoff_x =
    target.x - standoff_distance * std::cos(target.yaw);
  const double standoff_y =
    target.y - standoff_distance * std::sin(target.yaw);
  const double standoff_error = std::hypot(standoff_x, standoff_y);

  // 0=驶向预对接点，1=原地回正，2=沿目标轴压入。
  if (*phase == 0 &&
    standoff_error <= params_.body_alignment_standoff_tolerance)
  {
    *phase = 1;
  }
  if (*phase == 1 && heading_aligned) {
    *phase = 2;
  }
  if (*phase == 2 &&
    (center_distance > desired_center_distance + 0.22 ||
    std::abs(lateral) > 3.0 * params_.body_alignment_lateral_tolerance))
  {
    // 目标重新拉开距离或明显横移时，重新执行预对接。
    *phase = 0;
  }

  double angular = 0.0;
  if (*phase == 0) {
    // 阶段 1：先到目标后方的预对接点，避免过早回正造成大范围绕行。
    const double bearing = std::atan2(standoff_y, standoff_x);
    const double direction = std::abs(bearing) <= kPi / 2.0 ? 1.0 : -1.0;
    linear = direction * clip(
      std::max(std::abs(linear), 0.70 * standoff_error),
      0.02,
      params_.body_alignment_approach_speed);
    angular =
      direction * params_.body_alignment_bearing_gain * bearing -
      params_.body_alignment_yaw_rate_gain * ego_yaw_rate;
  } else if (*phase == 1) {
    // 阶段 2：已经到预对接点，停车并只执行航向回正。
    linear = 0.0;
    angular =
      params_.body_terminal_heading_gain * heading_error -
      params_.body_alignment_yaw_rate_gain * ego_yaw_rate;
  } else {
    // 阶段 3：航向已对齐，沿目标轴线低速压入。
    linear = clip(
      linear,
      -params_.body_alignment_speed,
      params_.body_alignment_speed);
    angular =
      params_.body_terminal_heading_gain * heading_error +
      0.5 * lateral -
      params_.body_alignment_yaw_rate_gain * ego_yaw_rate;
  }

  return fromTwist(linear, angular);
}

WheelCommand DifferentialController::computePostDockFollow(
  double ego_speed,
  const TargetState & target,
  double desired_center_distance,
  double ego_yaw_rate) const
{
  WheelCommand command;
  if (!target.initialized) {
    return command;
  }

  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);
  const double along = tx * target.x + ty * target.y;
  const double along_error = along - desired_center_distance;
  const double v_along = tx * target.vx + ty * target.vy;
  const double desired_v_along = clip(
    -params_.body_k_along * along_error,
    -params_.body_max_closure,
    params_.body_max_closure);
  double linear = ego_speed +
    params_.body_k_along_vel * (v_along - desired_v_along);
  linear = clip(linear, 0.0, params_.body_follow_max_speed);

  const double goal_x =
    target.x - desired_center_distance * std::cos(target.yaw);
  const double goal_y =
    target.y - desired_center_distance * std::sin(target.yaw);
  const double goal_distance = std::hypot(goal_x, goal_y);
  const double bearing = goal_distance > 0.03
    ? std::atan2(goal_y, goal_x)
    : 0.0;

  // 已对接后禁止把目标后方的点当成倒车目标。
  // 目标若向前移动，最终对接点仍在自车前方；若目标退到自车后方，
  // 先停车而不是突然反向，避免车尾朝前运动。
  if (goal_distance > 0.03 && goal_x < -0.02) {
    linear = 0.0;
  }

  double angular =
    params_.body_follow_heading_gain * normalizeAngle(target.yaw) +
    params_.body_follow_bearing_gain * bearing -
    params_.body_follow_yaw_rate_gain * ego_yaw_rate;
  angular = clip(
    angular,
    -params_.body_follow_max_angular,
    params_.body_follow_max_angular);

  return fromTwist(linear, angular);
}

WheelCommand DifferentialController::computeDocking(
  const Pose2D & ego_pose,
  const TargetState & target,
  double desired_center_distance) const
{
  WheelCommand command;
  if (!target.initialized) {
    return command;
  }

  const double dx = target.x - ego_pose.x;
  const double dy = target.y - ego_pose.y;
  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);

  const double longitudinal = tx * dx + ty * dy;
  const double lateral = -tx * dy + ty * dx;
  const double longitudinal_error = longitudinal - desired_center_distance;
  const double heading_error = normalizeAngle(target.yaw - ego_pose.yaw);

  // 闭合速度：自车仍位于对接位姿后方时为正。
  const double closure = clip(
    params_.docking_longitudinal_gain * longitudinal_error,
    -params_.docking_max_closure,
    params_.docking_max_closure);
  double linear = target.speed + closure;
  linear = std::max(0.0, linear);

  // 横向纠偏：朝目标中心线转向。
  double angular =
    target.omega +
    params_.docking_heading_gain * heading_error +
    params_.docking_lateral_gain * lateral;

  return fromBodyVelocity(linear, angular);
}

}  // 命名空间 docking
