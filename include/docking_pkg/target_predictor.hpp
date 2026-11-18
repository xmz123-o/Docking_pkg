#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

class TargetPredictor
{
public:
  struct Params
  {
    double prediction_dt{0.1};     // s
    double prediction_horizon{8.0};   // s
    double min_speed_for_ctrv{0.02};  // m/s
    double max_abs_omega{0.8};     // rad/s，目标合理转弯角速度上限
    double omega_deadband{0.04};   // rad/s
  };

  TargetPredictor();
  explicit TargetPredictor(const Params & params);

  TargetState propagate(const TargetState & state, double dt) const;
  Trajectory predict(const TargetState & state, double start_time) const;

  // 车体系相对预测：直接对估计的相对速度积分，而不是假设目标沿自身航向运动
  Trajectory predictRelative(const TargetState & state, double start_time) const;

private:
  Params params_;
};

}
