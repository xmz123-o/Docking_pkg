#include "docking_pkg/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "docking_pkg/quintic.hpp"

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

size_t nearestIndex(const Path & path, double x, double y)
{
  size_t best = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = squared(path[i].x - x) + squared(path[i].y - y);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

double signedLateralOffset(
  const PathPoint & reference,
  double x,
  double y)
{
  const double dx = x - reference.x;
  const double dy = y - reference.y;
  return -std::sin(reference.yaw) * dx + std::cos(reference.yaw) * dy;
}

double referenceKappaPrime(const Path & path, double s)
{
  if (path.size() < 2) {
    return 0.0;
  }

  size_t i = 0;
  while (i + 1 < path.size() && path[i + 1].s < s) {
    ++i;
  }

  if (i == 0) {
    const double ds = path[1].s - path[0].s;
    return ds > kEpsilon ? (path[1].kappa - path[0].kappa) / ds : 0.0;
  }
  if (i + 1 >= path.size()) {
    const double ds = path[i].s - path[i - 1].s;
    return ds > kEpsilon
      ? (path[i].kappa - path[i - 1].kappa) / ds
      : 0.0;
  }

  const double ds = path[i + 1].s - path[i - 1].s;
  return ds > kEpsilon
    ? (path[i + 1].kappa - path[i - 1].kappa) / ds
    : 0.0;
}

}  

LocalPlanner::LocalPlanner()
: LocalPlanner(Params{})
{
}

LocalPlanner::LocalPlanner(const Params & params)
: params_(params)
{
}

Path LocalPlanner::plan(const Pose2D & ego_pose, const Path & reference_line) const
{
  if (params_.mode == Mode::kFrenet) {
    return planFrenet(ego_pose, reference_line);
  }
  return planCartesian(ego_pose, reference_line);
}

Path LocalPlanner::planCartesian(
  const Pose2D & ego_pose, const Path & reference_line) const
{
  (void)ego_pose;
  return reference_line;
}

Path LocalPlanner::planFrenet(
  const Pose2D & ego_pose, const Path & reference_line) const
{
  if (reference_line.size() < 3) {
    return reference_line;
  }

  const size_t nearest = nearestIndex(reference_line, ego_pose.x, ego_pose.y);
  const PathPoint & nearest_point = reference_line[nearest];
  const double s0 = nearest_point.s;
  const double l0 = signedLateralOffset(nearest_point, ego_pose.x, ego_pose.y);
  const double s_end = reference_line.back().s;
  const double ds = s_end - s0;
  if (ds < 0.05) {
    return reference_line;
  }

  // 起始横向导数必须与自车实际航向一致，
  // 不能强制为 0。Frenet 坐标系下：
  //   tan(delta_theta) = l' / (1 - kappa_ref * l)
  const double delta_yaw = normalizeAngle(ego_pose.yaw - nearest_point.yaw);
  const double bounded_delta_yaw = clip(delta_yaw, -1.4, 1.4);
  double initial_A = 1.0 - nearest_point.kappa * l0;
  if (std::abs(initial_A) < 1e-3) {
    initial_A = initial_A < 0.0 ? -1e-3 : 1e-3;
  }
  const double dl0 = clip(
    initial_A * std::tan(bounded_delta_yaw), -5.0, 5.0);
  const double ddl0 = 0.0;

  const int samples = std::max(3, params_.sample_count);
  // 五次多项式在归一化坐标 u ∈ [0, 1] 下拟合，
  // 因此对 s 的导数需要先乘以 ds 再参与拟合。
  const auto lateral = fitQuintic(
    l0, dl0 * ds, ddl0 * ds * ds,
    0.0, 0.0, 0.0);

  Path path;
  path.reserve(static_cast<size_t>(samples + 1));
  double previous_x = ego_pose.x;
  double previous_y = ego_pose.y;
  double accumulated_s = 0.0;

  for (int i = 0; i <= samples; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(samples);
    const double s = s0 + u * ds;
    const double l = evalQuintic(lateral, u);
    const double dl_ds = evalQuintic(lateral, u, 1) / ds;
    const double ddl_ds2 = evalQuintic(lateral, u, 2) / (ds * ds);

    const PathPoint ref = interpolatePath(reference_line, s);
    const double kappa_ref = ref.kappa;
    const double kappa_ref_prime = referenceKappaPrime(reference_line, s);

    // 偏移曲线 p(s) = r(s) + n(s) * l(s) 的精确几何关系：
    //   A = 1 - kappa * l,   B = l'
    //   C = -kappa' * l - 2 * kappa * l'
    //   D = kappa * A + l''
    //   kappa_offset = (A * D - B * C) / (A^2 + B^2)^(3/2)
    double A = 1.0 - kappa_ref * l;
    if (std::abs(A) < 1e-6) {
      A = A < 0.0 ? -1e-6 : 1e-6;
    }
    const double B = dl_ds;
    const double C = -kappa_ref_prime * l - 2.0 * kappa_ref * dl_ds;
    const double D = kappa_ref * A + ddl_ds2;

    PathPoint point;
    point.x = ref.x - std::sin(ref.yaw) * l;
    point.y = ref.y + std::cos(ref.yaw) * l;
    point.yaw = normalizeAngle(ref.yaw + std::atan2(B, A));
    const double denominator = std::pow(A * A + B * B, 1.5);
    point.kappa = denominator > kEpsilon
      ? (A * D - B * C) / denominator
      : kappa_ref;

    if (i > 0) {
      accumulated_s += distance(previous_x, previous_y, point.x, point.y);
    }
    point.s = accumulated_s;
    path.push_back(point);

    previous_x = point.x;
    previous_y = point.y;
  }

  return path;
}

}  // 命名空间 docking
