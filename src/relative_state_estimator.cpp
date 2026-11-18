#include "docking_pkg/relative_state_estimator.hpp"

namespace docking
{

PositionProcessor::PositionProcessor() : PositionProcessor(Params{}) {}

PositionProcessor::PositionProcessor(const Params & params) : params_(params) {}

bool PositionProcessor::process(const QrPosition & qr,double target_yaw_estimate,
  Pose2D * target_in_ego) const
{
  if (!qr.valid || target_in_ego == nullptr) {
    return false;
  }

  // camera -> ego: p_ego = R(camera_yaw) * p_camera + camera_translation
  const double cc = std::cos(params_.camera_yaw);
  const double cs = std::sin(params_.camera_yaw);
  const double tag_x_ego = cc * qr.x - cs * qr.y + params_.camera_x;
  const double tag_y_ego = cs * qr.x + cc * qr.y + params_.camera_y;

  // tag -> target base: target center = tag origin -
  // R(target_yaw) * R(tag_yaw) * tag_offset。
  const double tc = std::cos(params_.tag_yaw);
  const double ts = std::sin(params_.tag_yaw);
  const double tag_offset_x = tc * params_.tag_x - ts * params_.tag_y;
  const double tag_offset_y = ts * params_.tag_x + tc * params_.tag_y;
  const double target_cos = std::cos(target_yaw_estimate);
  const double target_sin = std::sin(target_yaw_estimate);
  const double offset_x =
    target_cos * tag_offset_x - target_sin * tag_offset_y;
  const double offset_y =
    target_sin * tag_offset_x + target_cos * tag_offset_y;

  target_in_ego->x = tag_x_ego - offset_x;
  target_in_ego->y = tag_y_ego - offset_y;
  target_in_ego->yaw = target_yaw_estimate;

  const double range = std::hypot(target_in_ego->x, target_in_ego->y);
  if (!std::isfinite(target_in_ego->x) || !std::isfinite(target_in_ego->y) ||
    range < params_.min_range || range > params_.max_range)
  {
    return false;
  }
  return true;
}

EgoMotionCompensator::EgoMotionCompensator()
: EgoMotionCompensator(Params{})
{
}

EgoMotionCompensator::EgoMotionCompensator(const Params & params)
: params_(params)
{
  reset();
}

void EgoMotionCompensator::reset()
{
  previous_yaw_ = 0.0;
  previous_yaw_stamp_ = 0.0;
  have_previous_yaw_ = false;
  have_previous_ = false;
  previous_stamp_ = 0.0;
  previous_x_ = 0.0;
  previous_y_ = 0.0;
  ego_speed_ = 0.0;
  ego_omega_ = 0.0;
}

void EgoMotionCompensator::setEgoYaw(double stamp, double yaw)
{
  if (!have_previous_yaw_) {
    have_previous_yaw_ = true;
    previous_yaw_ = yaw;
    previous_yaw_stamp_ = stamp;
    ego_stamp_ = stamp;
    return;
  }

  const double dt = stamp - previous_yaw_stamp_;
  if (dt > 1e-3) {
    double raw_omega = normalizeAngle(yaw - previous_yaw_) / dt;
    raw_omega = clip(raw_omega, -params_.max_yaw_rate, params_.max_yaw_rate);
    if (std::abs(raw_omega) < params_.yaw_rate_deadband) {
      raw_omega = 0.0;
    }
    ego_omega_ = (1.0 - params_.yaw_rate_filter_alpha) * ego_omega_ +
      params_.yaw_rate_filter_alpha * raw_omega;
  }

  previous_yaw_ = yaw;
  previous_yaw_stamp_ = stamp;
  ego_stamp_ = stamp;
}

void EgoMotionCompensator::setEgoState(double stamp, double speed, double omega)
{
  ego_stamp_ = stamp;
  ego_speed_ = speed;
  ego_omega_ = omega;
}

void EgoMotionCompensator::compensate(
  const Pose2D & target_in_ego,
  double stamp,
  double * vx_relative,
  double * vy_relative)
{
  if (vx_relative == nullptr || vy_relative == nullptr) {
    return;
  }

  *vx_relative = 0.0;
  *vy_relative = 0.0;

  if (have_previous_) {
    const double dt = stamp - previous_stamp_;
    if (dt > 1e-3) {
      // p is measured in the rotating body frame. The relative velocity in the
      // body frame includes the -omega x p term from frame rotation.
      const double dp_x = (target_in_ego.x - previous_x_) / dt;
      const double dp_y = (target_in_ego.y - previous_y_) / dt;
      *vx_relative = dp_x - ego_omega_ * target_in_ego.y;
      *vy_relative = dp_y + ego_omega_ * target_in_ego.x;
    }
  }

  have_previous_ = true;
  previous_stamp_ = stamp;
  previous_x_ = target_in_ego.x;
  previous_y_ = target_in_ego.y;
}

TargetVelocityFilter::TargetVelocityFilter()
: TargetVelocityFilter(Params{})
{
}

TargetVelocityFilter::TargetVelocityFilter(const Params & params)
: params_(params)
{
  reset();
}

void TargetVelocityFilter::reset()
{
  state_ = TargetState{};
  previous_yaw_ = 0.0;
  have_previous_yaw_ = false;
}

void TargetVelocityFilter::update(
  double stamp,
  double x,
  double y,
  double vx_relative,
  double vy_relative,
  double ego_speed,
  double omega_hint)
{
  if (!state_.initialized) {
    state_.initialized = true;
    state_.stamp = stamp;
    state_.x = x;
    state_.y = y;
    state_.vx = 0.0;
    state_.vy = 0.0;
    state_.speed = 0.0;
    state_.yaw = 0.0;
    state_.omega = 0.0;
    return;
  }

  const double alpha = params_.alpha;
  state_.vx = (1.0 - alpha) * state_.vx + alpha * vx_relative;
  state_.vy = (1.0 - alpha) * state_.vy + alpha * vy_relative;
  if (std::abs(state_.vy) < 0.05) {
    state_.vy = 0.0;
  }

  double speed = std::hypot(state_.vx, state_.vy);
  if (speed > params_.max_speed) {
    const double scale = params_.max_speed / speed;
    state_.vx *= scale;
    state_.vy *= scale;
    speed = params_.max_speed;
  }

  state_.stamp = stamp;
  state_.x = x;
  state_.y = y;
  state_.speed = speed;

  // 第一版车体系模式假设目标沿当前局部 +X 直行。二维码位置和
  // 相对速度不能稳定给出全局目标航向，因此这里不包装成伪测量。
  (void)ego_speed;
  (void)omega_hint;
  state_.yaw = 0.0;
  state_.omega = 0.0;
}

}  // namespace docking
