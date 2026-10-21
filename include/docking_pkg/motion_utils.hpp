#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "docking_pkg/types.hpp"

namespace docking
{

struct TimedSpeedProfile
{
  bool valid{false};
  double peak_speed{0.0};
  double accel_time{0.0};
  double cruise_time{0.0};
  double decel_time{0.0};
};

inline double minimumTravelTime(
  double distance,
  double initial_speed,
  double final_speed,
  double max_accel,
  double max_decel,
  double max_speed)
{
  if (distance <= 1e-6) {
    return 0.0;
  }
  if (max_accel <= 1e-6 || max_decel <= 1e-6 || max_speed <= 1e-6) {
    return std::numeric_limits<double>::infinity();
  }

  const double v0 = clip(initial_speed, 0.0, max_speed);
  const double vf = clip(final_speed, 0.0, max_speed);
  const double vmax = max_speed;

  const double accel_distance = vmax > v0
    ? (vmax * vmax - v0 * v0) / (2.0 * max_accel)
    : 0.0;
  const double decel_distance = vmax > vf
    ? (vmax * vmax - vf * vf) / (2.0 * max_decel)
    : 0.0;

  if (accel_distance + decel_distance <= distance + 1e-9) {
    const double accel_time = std::max(0.0, (vmax - v0) / max_accel);
    const double decel_time = std::max(0.0, (vmax - vf) / max_decel);
    const double cruise_distance =
      std::max(0.0, distance - accel_distance - decel_distance);
    return accel_time + decel_time + cruise_distance / vmax;
  }

  const double denom = 1.0 / max_accel + 1.0 / max_decel;
  const double peak_sq =
    (2.0 * distance + v0 * v0 / max_accel + vf * vf / max_decel) / denom;
  const double peak_min = std::max(v0, vf);
  if (peak_sq < peak_min * peak_min - 1e-9) {
    return std::numeric_limits<double>::infinity();
  }

  const double peak = std::sqrt(std::max(0.0, peak_sq));
  if (peak > vmax + 1e-6) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, (peak - v0) / max_accel) +
    std::max(0.0, (peak - vf) / max_decel);
}

inline TimedSpeedProfile solveTimedSpeedProfile(
  double distance,
  double initial_speed,
  double final_speed,
  double target_time,
  double max_accel,
  double max_decel,
  double max_speed)
{
  TimedSpeedProfile profile;
  if (distance <= 1e-6) {
    profile.valid = true;
    profile.peak_speed = std::max(initial_speed, final_speed);
    return profile;
  }
  if (target_time <= 0.0 || max_accel <= 1e-6 || max_decel <= 1e-6 || max_speed <= 1e-6) {
    return profile;
  }

  const double minimum_time = minimumTravelTime(
    distance, initial_speed, final_speed, max_accel, max_decel, max_speed);
  if (!std::isfinite(minimum_time) || target_time < minimum_time - 1e-6) {
    return profile;
  }

  const double v0 = clip(initial_speed, 0.0, max_speed);
  const double vf = clip(final_speed, 0.0, max_speed);
  const double A = -(0.5 / max_accel + 0.5 / max_decel);
  const double B = target_time + v0 / max_accel + vf / max_decel;
  const double C =
    -(v0 * v0 / (2.0 * max_accel) + vf * vf / (2.0 * max_decel)) - distance;
  const double discriminant = B * B - 4.0 * A * C;
  if (discriminant < -1e-9) {
    return profile;
  }

  const double sqrt_disc = std::sqrt(std::max(0.0, discriminant));
  const double root1 = (-B + sqrt_disc) / (2.0 * A);
  const double root2 = (-B - sqrt_disc) / (2.0 * A);
  const double roots[2] = {std::min(root1, root2), std::max(root1, root2)};
  const double peak_min = std::max(v0, vf);

  for (const double peak : roots) {
    if (peak < peak_min - 1e-6 || peak > max_speed + 1e-6) {
      continue;
    }
    const double accel_time = (peak - v0) / max_accel;
    const double decel_time = (peak - vf) / max_decel;
    const double cruise_time = target_time - accel_time - decel_time;
    if (accel_time < -1e-6 || decel_time < -1e-6 || cruise_time < -1e-6) {
      continue;
    }
    profile.valid = true;
    profile.peak_speed = peak;
    profile.accel_time = std::max(0.0, accel_time);
    profile.decel_time = std::max(0.0, decel_time);
    profile.cruise_time = std::max(0.0, cruise_time);
    return profile;
  }

  return profile;
}

}  // 命名空间 docking
