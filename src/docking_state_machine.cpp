#include "docking_pkg/docking_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace docking
{

DockingStateMachine::DockingStateMachine()
: DockingStateMachine(Params{})
{
}

DockingStateMachine::DockingStateMachine(const Params & params)
: params_(params)
{
}

DockingState DockingStateMachine::update(
  const Pose2D & ego_pose,
  const TargetState & target,
  double ego_speed)
{
  state_ = DockingState{};

  if (!target.initialized) {
    state_.phase = DockingState::Phase::kWaitingForTarget;
    return state_;
  }

  const double dx = target.x - ego_pose.x;
  const double dy = target.y - ego_pose.y;
  state_.center_distance = std::hypot(dx, dy);
  state_.heading_error = std::abs(normalizeAngle(ego_pose.yaw - target.yaw));

  const double tx = std::cos(target.yaw);
  const double ty = std::sin(target.yaw);
  state_.longitudinal_error =
    tx * dx + ty * dy - params_.desired_center_distance;
  state_.lateral_error = -tx * dy + ty * dx;

  const double relative_speed_error = params_.body_frame_mode
    ? std::abs(tx * target.vx + ty * target.vy)
    : std::abs(ego_speed - target.speed);
  double closing_speed = clip(
    params_.catch_up_gain * state_.longitudinal_error,
    -params_.max_closure_speed,
    params_.max_closure_speed);
  if (state_.longitudinal_error > 0.02) {
    closing_speed = std::max(closing_speed, params_.min_closure_speed);
  } else if (state_.longitudinal_error < -0.02) {
    closing_speed = std::min(closing_speed, -params_.min_closure_speed);
  } else {
    closing_speed = 0.0;
  }
  state_.closing_speed = closing_speed;

  const auto catch_up_speed = [&](double configured_minimum) {
    const double desired = target.speed + closing_speed;
    return clip(
      std::max(configured_minimum, desired), 0.0, params_.max_ego_speed);
  };

  bool docked = false;
  if (docked_latch_) {
    const double minimum_docked_distance = std::max(
      0.0, params_.docked_distance - params_.docked_release_margin);
    docked =
      state_.center_distance <
        params_.docked_distance + params_.docked_release_margin &&
      state_.center_distance > minimum_docked_distance &&
      std::abs(state_.lateral_error) <
        params_.docked_lateral_error + params_.docked_release_margin &&
      state_.heading_error <
        params_.docked_heading_error + params_.docked_release_margin;
    if (!docked) {
      docked_latch_ = false;
    }
  }

  if (!docked_latch_ &&
    state_.center_distance < params_.docked_distance &&
    std::abs(state_.longitudinal_error) < params_.docked_longitudinal_error &&
    relative_speed_error < params_.docked_speed &&
    state_.heading_error < params_.docked_heading_error &&
    std::abs(state_.lateral_error) < params_.docked_lateral_error)
  {
    docked_latch_ = true;
    docked = true;
  }

  if (docked) {
    state_.phase = DockingState::Phase::kDocked;
    if (state_.longitudinal_error > params_.docked_longitudinal_error) {
      state_.cruise_speed = clip(
        target.speed + params_.final_closure_speed,
        0.0, params_.max_ego_speed);
    } else {
      state_.cruise_speed = clip(target.speed, 0.0, params_.max_ego_speed);
    }
    state_.final_speed = state_.cruise_speed;
  } else if (state_.center_distance < params_.final_dock_distance &&
    state_.heading_error < params_.final_heading_error &&
    std::abs(state_.lateral_error) < params_.final_lateral_error)
  {
    state_.phase = DockingState::Phase::kFinalDock;
    if (state_.longitudinal_error > 0.0) {
      state_.cruise_speed = clip(
        std::max(params_.final_speed, target.speed + params_.final_closure_speed),
        0.0, params_.max_ego_speed);
    } else {
      state_.cruise_speed = clip(target.speed, 0.0, params_.max_ego_speed);
    }
    state_.final_speed = clip(target.speed, 0.0, params_.max_ego_speed);
  } else if (state_.center_distance < params_.align_distance) {
    state_.phase = DockingState::Phase::kAlign;
    state_.cruise_speed = catch_up_speed(params_.align_speed);
    state_.final_speed = clip(target.speed, 0.0, params_.max_ego_speed);
  } else if (state_.center_distance < params_.approach_distance) {
    state_.phase = DockingState::Phase::kApproach;
    state_.cruise_speed = catch_up_speed(params_.approach_speed);
    state_.final_speed = clip(target.speed, 0.0, params_.max_ego_speed);
  } else if (state_.center_distance < params_.tracking_distance) {
    state_.phase = DockingState::Phase::kTracking;
    state_.cruise_speed = catch_up_speed(params_.tracking_speed);
    state_.final_speed = clip(target.speed, 0.0, params_.max_ego_speed);
  } else {
    state_.phase = DockingState::Phase::kTracking;
    state_.cruise_speed = catch_up_speed(params_.tracking_speed);
    state_.final_speed = clip(target.speed, 0.0, params_.max_ego_speed);
  }

  return state_;
}

std::string DockingStateMachine::phaseName(DockingState::Phase phase)
{
  switch (phase) {
    case DockingState::Phase::kWaitingForTarget:
      return "WAITING_FOR_TARGET";
    case DockingState::Phase::kTracking:
      return "TRACKING";
    case DockingState::Phase::kApproach:
      return "APPROACH";
    case DockingState::Phase::kAlign:
      return "ALIGN";
    case DockingState::Phase::kFinalDock:
      return "FINAL_DOCK";
    case DockingState::Phase::kDocked:
      return "DOCKED";
  }
  return "UNKNOWN";
}

}  // 命名空间 docking
