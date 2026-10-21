#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

class SpeedPlanner
{
public:
  struct Params
  {
    double max_accel{0.8};
    double max_decel{1.0};
    double integration_dt{0.02};
    int max_iterations{20000};
  };

  SpeedPlanner();
  explicit SpeedPlanner(const Params & params);

  Trajectory plan(
    const Path & path,
    double initial_speed,
    double cruise_speed,
    double final_speed,
    double target_time = -1.0) const;

private:
  Params params_;
};

}  // 命名空间 docking
