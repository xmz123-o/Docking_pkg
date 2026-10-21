#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace docking
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double s{0.0};
  double kappa{0.0};
};

struct TrajectoryPoint
{
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double a{0.0};
  double omega{0.0};
  double kappa{0.0};
  double s{0.0};
};

struct DockingGoal
{
  bool valid{false};
  double time{0.0};  // 与预测使用同一时钟的绝对时间
  Pose2D pose;
  double speed{0.0};
  double omega{0.0};
};

struct TargetState
{
  bool initialized{false};
  double stamp{0.0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double ax{0.0};
  double ay{0.0};
  double yaw{0.0};
  double omega{0.0};
  double speed{0.0};
};

using Path = std::vector<PathPoint>;
using Trajectory = std::vector<TrajectoryPoint>;

struct ReferenceLine
{
  Path path;
  DockingGoal docking_goal;
};

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

inline double distance(double x0, double y0, double x1, double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return std::hypot(dx, dy);
}

inline double clip(double value, double low, double high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

inline double squared(double value)
{
  return value * value;
}

}  // 命名空间 docking
