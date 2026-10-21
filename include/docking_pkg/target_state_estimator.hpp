#pragma once

#include <deque>
#include <utility>

#include <Eigen/Dense>

#include "docking_pkg/types.hpp"

namespace docking
{

class TargetStateEstimator
{
public:
  struct Params
  {
    double measurement_noise_position{0.0001};  // 平方米
    double process_noise_position{0.02};
    double process_noise_velocity{0.05};
    double max_plausible_velocity{15.0};  // 米/秒
    double min_speed_for_heading{0.08};  // 米/秒
    double omega_window_time{2.5};       // 秒，滑动窗口回归
    double omega_filter_alpha{0.08};     // 0~1，横摆角速度估计低通滤波系数
    double omega_deadband{0.04};         // 弧度/秒
    double max_plausible_omega{0.8};     // 弧度/秒
    bool use_measured_orientation{true};
  };

  TargetStateEstimator();
  explicit TargetStateEstimator(const Params & params);

  void reset();
  void predict(double dt);
  void update(double stamp, double x, double y);
  void setYaw(double yaw);
  void updateVelocity(
    double stamp,
    double vx,
    double vy,
    double omega,
    bool has_omega);

  const TargetState & state() const { return state_; }
  TargetState & state() { return state_; }
  bool useMeasuredOrientation() const { return params_.use_measured_orientation; }

private:
  Params params_;
  TargetState state_;

  //卡尔曼滤波器内部状态: x, y, vx, vy
  Eigen::Vector4d x_;

  //状态协方差矩阵
  Eigen::Matrix4d P_;

  // (时间戳, 展开航向角) 滑动窗口，用于估计
  // 横摆角速度，避免对两帧含噪测量直接差分。
  std::deque<std::pair<double, double>> yaw_history_;
  bool have_last_yaw_{false};
  double last_yaw_raw_{0.0};
  double last_yaw_unwrapped_{0.0};

  double last_update_time_{0.0};
  bool have_last_update_time_{false};
};

}  // 命名空间 docking
