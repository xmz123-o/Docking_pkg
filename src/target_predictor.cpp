#include "docking_pkg/target_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace docking
{

TargetPredictor::TargetPredictor()
: TargetPredictor(Params{})
{
}

TargetPredictor::TargetPredictor(const Params & params)
: params_(params)
{
}

TargetState TargetPredictor::propagate(const TargetState & state, double dt) const
{
  TargetState out = state;
  if (!state.initialized || dt <= 0.0) {
    return out;
  }

  const double v = std::max(0.0, state.speed);
  double yaw = state.yaw;
  double omega = state.omega;


  if (v < params_.min_speed_for_ctrv) {
    omega = 0.0;
  }

  if (std::abs(omega) < 1e-6) {
    out.x = state.x + v * std::cos(yaw) * dt;
    out.y = state.y + v * std::sin(yaw) * dt;
  } else {
    const double yaw_next = yaw + omega * dt;
    out.x = state.x + (v / omega) * (std::sin(yaw_next) - std::sin(yaw));
    out.y = state.y - (v / omega) * (std::cos(yaw_next) - std::cos(yaw));
    yaw = normalizeAngle(yaw_next);
  }

  out.yaw = yaw;
  out.omega = omega;
  out.vx = v * std::cos(yaw);
  out.vy = v * std::sin(yaw);
  out.speed = v;
  out.stamp = state.stamp + dt;
  return out;
}

Trajectory TargetPredictor::predict(const TargetState & state, double start_time) const
{
  Trajectory trajectory;
  if (!state.initialized) {
    return trajectory;
  }

  const double dt = std::max(0.01, params_.prediction_dt);
  const double horizon = std::max(dt, params_.prediction_horizon);
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
  trajectory.reserve(static_cast<size_t>(steps + 1));

  TrajectoryPoint start;
  start.t = start_time;
  start.x = state.x;
  start.y = state.y;
  start.yaw = state.yaw;
  start.v = state.speed;
  start.a = 0.0;
  start.omega = state.speed > params_.min_speed_for_ctrv ? state.omega : 0.0;
  start.kappa = start.v > params_.min_speed_for_ctrv ? start.omega / start.v : 0.0;
  start.s = 0.0;
  trajectory.push_back(start);

  double previous_x = start.x;
  double previous_y = start.y;
  double accumulated_s = 0.0;

  for (int i = 1; i <= steps; ++i) {
    const double elapsed = std::min(horizon, i * dt);
    const TargetState predicted = propagate(state, elapsed);
    accumulated_s += distance(previous_x, previous_y, predicted.x, predicted.y);

    TrajectoryPoint point;
    point.t = start_time + elapsed;
    point.x = predicted.x;
    point.y = predicted.y;
    point.yaw = predicted.yaw;
    point.v = predicted.speed;
    point.a = 0.0;
    point.omega = predicted.omega;
    point.kappa = point.v > params_.min_speed_for_ctrv
      ? predicted.omega / point.v
      : 0.0;
    point.s = accumulated_s;
    trajectory.push_back(point);

    previous_x = predicted.x;
    previous_y = predicted.y;
    if (elapsed >= horizon - 1e-9) {
      break;
    }
  }

  return trajectory;
}

}  

namespace docking
{

Trajectory TargetPredictor::predictRelative(
  const TargetState & state, double start_time) const
{
  Trajectory trajectory;
  if (!state.initialized) {
    return trajectory;
  }

  const double dt = std::max(0.01, params_.prediction_dt);
  const double horizon = std::max(dt, params_.prediction_horizon);
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
  trajectory.reserve(static_cast<size_t>(steps + 1));

  TrajectoryPoint start;
  start.t = start_time;
  start.x = state.x;
  start.y = state.y;
  start.yaw = state.yaw;
  start.v = state.speed;
  start.a = 0.0;
  start.omega = state.omega;
  start.kappa = state.speed > params_.min_speed_for_ctrv
    ? state.omega / state.speed
    : 0.0;
  start.s = 0.0;
  trajectory.push_back(start);

  double previous_x = state.x;
  double previous_y = state.y;
  double accumulated_s = 0.0;

  for (int i = 1; i <= steps; ++i) {
    const double elapsed = std::min(horizon, i * dt);
    TrajectoryPoint point;
    point.t = start_time + elapsed;
    point.x = state.x + state.vx * elapsed;
    point.y = state.y + state.vy * elapsed;
    point.yaw = normalizeAngle(state.yaw + state.omega * elapsed);
    point.v = state.speed;
    point.a = 0.0;
    point.omega = state.omega;
    point.kappa = state.speed > params_.min_speed_for_ctrv
      ? state.omega / state.speed
      : 0.0;
    accumulated_s += distance(previous_x, previous_y, point.x, point.y);
    point.s = accumulated_s;
    trajectory.push_back(point);
    previous_x = point.x;
    previous_y = point.y;
  }

  return trajectory;
}

}  // 命名空间 docking
