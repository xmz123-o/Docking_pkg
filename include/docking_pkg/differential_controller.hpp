#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

struct WheelCommand
{
  double left_rad_s{0.0};
  double right_rad_s{0.0};
  double linear{0.0};
  double angular{0.0};
};

class DifferentialController
{
public:
  struct Params
  {
    double wheel_radius{0.254};  // 10 英寸，换算为米
    double wheel_track{0.60};
    double max_linear{1.0};
    double max_angular{1.2};
    double max_wheel_rad_s{8.0};
    double lookahead_distance{0.12};
    double lookahead_time{0.30};
    double max_lookahead_distance{0.30};
    double path_heading_gain{2.5};
    double path_feedforward_gain{0.8};
    double docking_longitudinal_gain{1.5};
    double docking_lateral_gain{1.5};
    double docking_heading_gain{2.5};
    double docking_max_closure{0.25};
    double body_k_along{1.2};
    double body_k_along_vel{0.8};
    double body_k_heading{2.5};
    double body_k_lateral{1.5};
    double body_k_lateral_vel{0.6};
    double body_max_closure{0.30};
  };

  DifferentialController();
  explicit DifferentialController(const Params & params);

  WheelCommand compute(
    const Pose2D & ego_pose,
    const Trajectory & trajectory,
    double cruise_speed) const;

  // 末端使用的直接相对位姿对接控制，将自车调节到
  // 目标中心线上的对接位姿，而不是仅
  // 跟踪预计算路径，从而消除稳态横向误差。
  WheelCommand computeDocking(
    const Pose2D & ego_pose,
    const TargetState & target,
    double desired_center_distance) const;

  WheelCommand fromTwist(double linear, double angular) const;

  // 车体系相对对接控制器。目标状态表示在
  // 自车车体系中，target.vx/vy 是相对速度。
  WheelCommand computeBodyRelative(
    double ego_speed,
    const TargetState & target,
    double desired_center_distance) const;

private:
  WheelCommand fromBodyVelocity(double linear, double angular) const;

  Params params_;
};

}  // 命名空间 docking
