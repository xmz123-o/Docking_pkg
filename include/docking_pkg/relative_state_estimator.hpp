#pragma once

#include <algorithm>
#include <cmath>

#include "docking_pkg/types.hpp"

namespace docking
{

struct QrPosition
{
  bool valid{false};
  double stamp{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// 模块 1：相机坐标系 -> 自车坐标系、标签安装偏移补偿
// 和异常值剔除。二维码只提供位置，不提供目标航向。
class PositionProcessor
{
public:
  struct Params
  {
    double camera_x{0.35};
    double camera_y{0.0};
    double camera_yaw{0.0};
    double tag_x{-0.26};
    double tag_y{0.0};
    double tag_yaw{0.0};
    double min_range{0.05};
    double max_range{50.0};
  };

  PositionProcessor();
  explicit PositionProcessor(const Params & params);

  bool process(
    const QrPosition & qr,
    double target_yaw_estimate,
    Pose2D * target_in_ego) const;

  const Params & params() const { return params_; }

private:
  Params params_;
};

// 模块 2：补偿自车运动和车体系旋转。测量得到的
// 相对位置在当前自车车体系中；相对速度
// 必须包含车体系旋转产生的 -omega × p 项。
class EgoMotionCompensator
{
public:
  struct Params
  {
    double yaw_rate_filter_alpha{0.2};
    double max_yaw_rate{2.0};
    double yaw_rate_deadband{0.01};
  };

  EgoMotionCompensator();
  explicit EgoMotionCompensator(const Params & params);

  void reset();
  void setParams(const Params & params) { params_ = params; }

  // IMU 提供横摆角，横摆角速度由横摆角差分
  // 并经过低通滤波得到。
  void setEgoYaw(double stamp, double yaw);
  void setEgoState(double stamp, double speed, double omega);

  void compensate(
    const Pose2D & target_in_ego,
    double stamp,
    double * vx_relative,
    double * vy_relative);

  double egoSpeed() const { return ego_speed_; }
  double egoOmega() const { return ego_omega_; }

private:
  Params params_;
  double previous_yaw_{0.0};
  double previous_yaw_stamp_{0.0};
  bool have_previous_yaw_{false};

  double ego_stamp_{0.0};
  double ego_speed_{0.0};
  double ego_omega_{0.0};
  bool have_previous_{false};
  double previous_stamp_{0.0};
  double previous_x_{0.0};
  double previous_y_{0.0};
};

// 模块 3：对目标相对速度滤波并输出相对 TargetState。
class TargetVelocityFilter
{
public:
  struct Params
  {
    double alpha{0.25};
    double max_speed{8.0};
    double max_omega{2.0};
    double min_speed_for_heading{0.03};
  };

  TargetVelocityFilter();
  explicit TargetVelocityFilter(const Params & params);

  void reset();
  void update(
    double stamp,
    double x,
    double y,
    double vx_relative,
    double vy_relative,
    double ego_speed,
    double omega_hint);

  const TargetState & state() const { return state_; }

private:
  Params params_;
  TargetState state_;
  double previous_yaw_{0.0};
  double previous_stamp_{0.0};
  bool have_previous_yaw_{false};
};

}  // 命名空间 docking
