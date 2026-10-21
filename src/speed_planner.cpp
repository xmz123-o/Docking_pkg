#include "docking_pkg/speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "docking_pkg/motion_utils.hpp"

namespace docking
{

namespace
{

constexpr double kEpsilon = 1e-9;

PathPoint interpolatePath(const Path & path, double s)
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
      const double ratio = (s1 - s0) > kEpsilon ? (s - s0) / (s1 - s0) : 0.0;
      PathPoint out;
      out.x = path[i - 1].x + ratio * (path[i].x - path[i - 1].x);
      out.y = path[i - 1].y + ratio * (path[i].y - path[i - 1].y);
      out.yaw = normalizeAngle(path[i - 1].yaw +
        ratio * normalizeAngle(path[i].yaw - path[i - 1].yaw));
      out.kappa = path[i - 1].kappa + ratio * (path[i].kappa - path[i - 1].kappa);
      out.s = s;
      return out;
    }
  }
  return path.back();
}

TrajectoryPoint makePoint(
  const PathPoint & path_point,
  double t,
  double v,
  double a)
{
  TrajectoryPoint point;
  point.t = t;
  point.x = path_point.x;
  point.y = path_point.y;
  point.yaw = path_point.yaw;
  point.v = v;
  point.a = a;
  point.kappa = path_point.kappa;
  point.omega = v * path_point.kappa;
  point.s = path_point.s;
  return point;
}

Trajectory buildTimedTrajectory(
  const Path & path,
  double initial_speed,
  double final_speed,
  double max_accel,
  double max_decel,
  const TimedSpeedProfile & profile,
  double integration_dt,
  int max_iterations)
{
  Trajectory trajectory;

  if (path.size() < 2 || !profile.valid) {
    return trajectory;
  }

  const double total_length = path.back().s;
  const double target_time =
    profile.accel_time + profile.cruise_time + profile.decel_time;

  if (total_length <= kEpsilon || target_time <= kEpsilon) {
    return trajectory;
  }

  const double v0 = std::max(0.0, initial_speed);
  const double vf = std::max(0.0, final_speed);
  const double dt = std::max(0.005, integration_dt);

  const double accel_end = profile.accel_time;
  const double cruise_end =
    profile.accel_time + profile.cruise_time;

  // 检查速度规划阶段是否有效。
  if (profile.accel_time < 0.0 ||
      profile.cruise_time < 0.0 ||
      profile.decel_time < 0.0 ||
      profile.peak_speed < std::max(v0, vf) - 1e-6) {
    return trajectory;
  }

  trajectory.reserve(
    static_cast<size_t>(std::ceil(target_time / dt)) + 2);

  trajectory.push_back(
    makePoint(path.front(), 0.0, v0, 0.0));

  double t = 0.0;
  double s = 0.0;
  double v = v0;

  for (int i = 0; i < max_iterations; ++i) {
    if (t >= target_time - 1e-9) {
      break;
    }

    // 时间步长不能越过加速、巡航、减速阶段的分界点。
    double next_boundary = target_time;

    if (t < accel_end - 1e-9) {
      next_boundary = std::min(next_boundary, accel_end);
    } else if (t < cruise_end - 1e-9) {
      next_boundary = std::min(next_boundary, cruise_end);
    }

    const double dt_step =
      std::min(dt, next_boundary - t);

    if (dt_step <= kEpsilon) {
      return Trajectory{};
    }

    const double t_next = t + dt_step;
    double v_next = 0.0;

    if (t_next <= accel_end + 1e-9) {
      v_next = std::min(
        v0 + max_accel * t_next,
        profile.peak_speed);
    } else if (t_next <= cruise_end + 1e-9) {
      v_next = profile.peak_speed;
    } else {
      v_next = std::max(
        0.0,
        profile.peak_speed -
          max_decel * (t_next - cruise_end));
    }

    // 不通过强制赋值来掩盖不合理的末端速度。
    if (!std::isfinite(v_next)) {
      return Trajectory{};
    }

    // 梯形积分。
    const double ds = 0.5 * (v + v_next) * dt_step;
    const double s_next = s + ds;

    // 如果速度曲线在目标时间之前就超出了路径，
    // 说明当前速度剖面与路径长度不匹配。
    if (s_next > total_length + 1e-5) {
      return Trajectory{};
    }

    const double acceleration =
      (v_next - v) / dt_step;

    // 检查实际速度变化对应的加速度，而不是只裁剪记录值。
    if (acceleration > max_accel + 1e-4 ||
        acceleration < -max_decel - 1e-4) {
      return Trajectory{};
    }

    s = std::min(s_next, total_length);
    t = t_next;
    v = v_next;

    const PathPoint ref = interpolatePath(path, s);
    trajectory.push_back(
      makePoint(ref, t, v, acceleration));
  }

  if (trajectory.empty()) {
    return trajectory;
  }

  const TrajectoryPoint & last = trajectory.back();

  // 必须同时满足终端时间、终端位置和终端速度要求。
  // 不满足就返回空轨迹，由上层明确处理失败。
  const double position_error =
    std::abs(total_length - last.s);

  const double time_error =
    std::abs(target_time - last.t);

  const double speed_error =
    std::abs(vf - last.v);

  if (position_error > 0.03 ||
      time_error > std::max(0.02, dt) ||
      speed_error > 0.05) {
    return Trajectory{};
  }

  // 只对已经通过校验的轨迹做微小数值误差修正。
  TrajectoryPoint & final_point = trajectory.back();
  final_point.x = path.back().x;
  final_point.y = path.back().y;
  final_point.yaw = path.back().yaw;
  final_point.s = total_length;
  final_point.v = vf;
  final_point.kappa = path.back().kappa;
  final_point.omega = vf * path.back().kappa;

  return trajectory;
}

Trajectory buildStoppingTrajectory(
  const Path & path,
  double initial_speed,
  double final_speed,
  double max_speed,
  double max_accel,
  double max_decel,
  double integration_dt,
  int max_iterations)
{
  Trajectory trajectory;
  if (path.empty()) {
    return trajectory;
  }

  const double total_length = path.back().s;
  const double dt = std::max(0.005, integration_dt);
  const double v0 = clip(initial_speed, 0.0, max_speed);
  const double vf = clip(final_speed, 0.0, max_speed);

  trajectory.push_back(makePoint(
    path.front(), 0.0, v0, v0 < max_speed ? max_accel : 0.0));
  if (total_length < 1e-4) {
    return trajectory;
  }

  double t = 0.0;
  double s = 0.0;
  double v = v0;

  for (int i = 0; i < max_iterations; ++i) {
    if (s >= total_length - 1e-6) {
      break;
    }

    const double remaining = std::max(0.0, total_length - s);
    // 当前允许速度，保证车辆仍能在
    // 路径终点减速到 vf，这是二阶制动约束。
    const double brake_speed = std::sqrt(
      std::max(0.0, vf * vf + 2.0 * max_decel * remaining));
    const double acceleration_speed = v + max_accel * dt;
    double v_next = std::min({max_speed, brake_speed, acceleration_speed});
    v_next = std::max(0.0, v_next);
    if (v_next <= 1e-6 && remaining > 1e-3) {
      break;
    }

    double ds = v_next * dt;
    double dt_step = dt;
    if (ds >= remaining) {
      ds = remaining;
      dt_step = v_next > kEpsilon ? ds / v_next : dt;
    }

    const double previous_v = v;
    s = std::min(total_length, s + ds);
    t += dt_step;
    v = (s >= total_length - 1e-9) ? vf : v_next;

    double acceleration = dt_step > kEpsilon ? (v - previous_v) / dt_step : 0.0;
    acceleration = clip(acceleration, -max_decel, max_accel);

    const PathPoint ref = interpolatePath(path, s);
    trajectory.push_back(makePoint(ref, t, v, acceleration));
  }

  if (trajectory.back().s < total_length - 1e-3) {
    trajectory.back().v = 0.0;
    trajectory.back().a = 0.0;
    trajectory.back().omega = 0.0;
    return trajectory;
  }

  TrajectoryPoint & last = trajectory.back();
  last.x = path.back().x;
  last.y = path.back().y;
  last.yaw = path.back().yaw;
  last.s = total_length;
  last.v = vf;
  last.kappa = path.back().kappa;
  last.omega = vf * path.back().kappa;
  return trajectory;
}

}  // 命名空间

SpeedPlanner::SpeedPlanner()
: SpeedPlanner(Params{})
{
}

SpeedPlanner::SpeedPlanner(const Params & params)
: params_(params)
{
}

Trajectory SpeedPlanner::plan(
  const Path & path,
  double initial_speed,
  double cruise_speed,
  double final_speed,
  double target_time) const
{
  if (path.empty()) {
    return Trajectory{};
  }

  const double max_accel = std::max(1e-3, params_.max_accel);
  const double max_decel = std::max(1e-3, params_.max_decel);
  const double max_speed = std::max(
    0.0, std::max(cruise_speed, std::max(initial_speed, final_speed)));
  const double total_length = path.back().s;
  const double v0 = clip(initial_speed, 0.0, max_speed);
  const double vf = clip(final_speed, 0.0, max_speed);

  if (total_length < 1e-4) {
    Trajectory trajectory;
    trajectory.push_back(makePoint(path.front(), 0.0, v0, 0.0));
    trajectory.back().v = vf;
    return trajectory;
  }

  if (std::isfinite(target_time) && target_time > 0.0) {
    const TimedSpeedProfile profile = solveTimedSpeedProfile(
      total_length, v0, vf, target_time, max_accel, max_decel, max_speed);
    if (profile.valid) {
      Trajectory trajectory = buildTimedTrajectory(
        path, v0, vf, max_accel, max_decel, profile,
        params_.integration_dt, params_.max_iterations);
      if (!trajectory.empty()) {
        return trajectory;
      }
    }
  }

  return buildStoppingTrajectory(
    path, v0, vf, max_speed, max_accel, max_decel,
    params_.integration_dt, params_.max_iterations);
}

}  // 命名空间 docking
