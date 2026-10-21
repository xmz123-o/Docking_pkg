#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

class TargetPredictor
{
public:
  struct Params
  {
    double prediction_dt{0.1};     // 秒
    double prediction_horizon{8.0};   // 秒
    double min_speed_for_ctrv{0.02};  // 米/秒
    double max_abs_omega{0.8};     // 弧度/秒，目标合理转弯角速度上限
    double omega_deadband{0.04};   // 弧度/秒
  };

  TargetPredictor();
  explicit TargetPredictor(const Params & params);

  TargetState propagate(const TargetState & state, double dt) const;
  Trajectory predict(const TargetState & state, double start_time) const;

  // 车体系相对预测：直接对估计的相对速度积分，
  // 而不是假设目标沿自身航向运动。
  Trajectory predictRelative(const TargetState & state, double start_time) const;

private:
  Params params_;
};

}  // 命名空间 docking
