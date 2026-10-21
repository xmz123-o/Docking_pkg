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
  linear = std::max(0.0, linear);

  double angular =
    target.omega +
    1.0 * normalizeAngle(target.yaw) -
    0.5 * lateral -
    0.1 * v_lateral;

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
