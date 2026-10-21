#pragma once

#include "docking_pkg/types.hpp"

namespace docking
{

class ReferenceLineGenerator
{
public:
  struct Params
  {
    double target_length{0.52};
    double ego_length{0.52};
    double cone_bus_length{0.08};
    double docking_depth{0.05};
    int sample_count{80};
    double derivative_scale{1.0};
    double max_ego_speed{1.0};
    double max_ego_accel{0.8};
    double max_ego_decel{1.0};
    double min_docking_time{0.2};
    double max_docking_time{10.0};
    double prealign_distance{0.2};
    double prealign_activation_distance{1.2};
    double desired_center_distance{0.63};
    double docking_goal_lead{0.08};
    double path_length_margin{1.05};
  };

  ReferenceLineGenerator();
  explicit ReferenceLineGenerator(const Params & params);

  ReferenceLine generate(
    const Pose2D & ego_pose,
    double ego_speed,
    const Trajectory & predicted_target) const;

  double desired_center_distance() const
  {
    const double geometric =
      params_.target_length / 2.0 + params_.ego_length / 2.0 +
      2.0 * params_.cone_bus_length - params_.docking_depth;
    return params_.desired_center_distance > 0.0
      ? params_.desired_center_distance
      : geometric;
  }

private:
  Params params_;
};

}  // 命名空间 docking
