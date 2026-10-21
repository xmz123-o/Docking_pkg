#pragma once

#include <string>

#include "docking_pkg/types.hpp"

namespace docking
{

struct DockingState
{
  enum class Phase
  {
    kWaitingForTarget,
    kTracking,
    kApproach,
    kAlign,
    kFinalDock,
    kDocked,
  };

  Phase phase{Phase::kWaitingForTarget};
  double cruise_speed{0.0};
  double final_speed{0.0};
  double center_distance{0.0};
  double longitudinal_error{0.0};
  double lateral_error{0.0};
  double heading_error{0.0};
  double closing_speed{0.0};
};

class DockingStateMachine
{
public:
  struct Params
  {
    double tracking_distance{2.0};
    double approach_distance{1.0};
    double align_distance{0.95};
    double final_dock_distance{0.75};
    double docked_distance{0.64};
    double docked_speed{0.08};
    double final_heading_error{0.35};
    double docked_heading_error{0.25};
    double final_lateral_error{0.20};
    double docked_lateral_error{0.15};
    double tracking_speed{1.0};
    double approach_speed{0.6};
    double align_speed{0.35};
    double final_speed{0.12};
    double docked_hold_speed{0.0};
    double desired_center_distance{0.52};
    double docked_longitudinal_error{0.02};
    double final_closure_speed{0.04};
    double docked_release_margin{0.30};
    double max_ego_speed{1.0};
    double catch_up_gain{0.8};
    double max_closure_speed{0.6};
    double min_closure_speed{0.03};
    bool body_frame_mode{false};
  };

  DockingStateMachine();
  explicit DockingStateMachine(const Params & params);

  DockingState update(
    const Pose2D & ego_pose,
    const TargetState & target,
    double ego_speed);

  const DockingState & state() const { return state_; }

  static std::string phaseName(DockingState::Phase phase);

private:
  Params params_;
  DockingState state_;
  bool docked_latch_{false};
};

}  // 命名空间 docking
