#include "docking_pkg/target_state_estimator.hpp"

#include <cmath>

namespace docking
{

TargetStateEstimator::TargetStateEstimator(): TargetStateEstimator(Params{}) {}

TargetStateEstimator::TargetStateEstimator(const Params & params): params_(params)
{
  reset();
}

void TargetStateEstimator::reset()
{
  state_ = TargetState{};
  x_ = Eigen::Vector4d::Zero();
  P_ = Eigen::Matrix4d::Identity();
  P_(0, 0) = 1.0;
  P_(1, 1) = 1.0;
  P_(2, 2) = 4.0;
  P_(3, 3) = 4.0;
  yaw_history_.clear();
  have_last_yaw_ = false;
  last_yaw_raw_ = 0.0;
  last_yaw_unwrapped_ = 0.0;
  have_last_update_time_ = false;
  last_update_time_ = 0.0;
}

void TargetStateEstimator::predict(double dt)
{
  if (dt <= 0.0 || !state_.initialized) {
    return;
  }

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Q(0, 0) = dt4 / 4.0 * params_.process_noise_velocity;
  Q(0, 2) = dt3 / 2.0 * params_.process_noise_velocity;
  Q(1, 1) = Q(0, 0);
  Q(1, 3) = Q(0, 2);
  Q(2, 0) = Q(0, 2);
  Q(2, 2) = dt2 * params_.process_noise_velocity;
  Q(3, 1) = Q(0, 2);
  Q(3, 3) = Q(2, 2);

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;

  state_.x = x_(0);
  state_.y = x_(1);
  state_.vx = x_(2);
  state_.vy = x_(3);
  state_.speed = std::hypot(state_.vx, state_.vy);
  state_.yaw = state_.speed > params_.min_speed_for_heading
    ? std::atan2(state_.vy, state_.vx)
    : state_.yaw;
}

void TargetStateEstimator::update(double stamp, double x, double y)
{
  if (!state_.initialized) {
    state_.initialized = true;
    state_.stamp = stamp;
    state_.x = x;
    state_.y = y;
    x_ << x, y, 0.0, 0.0;
    have_last_update_time_ = true;
    last_update_time_ = stamp;
    return;
  }

  const double dt = stamp - last_update_time_;
  const double previous_vx = state_.vx;
  const double previous_vy = state_.vy;
  if (dt > 0.0) {
    predict(dt);
  }
  last_update_time_ = stamp;
  have_last_update_time_ = true;

  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * params_.measurement_noise_position;

  const Eigen::Matrix<double, 2, 1> z(x, y);
  const Eigen::Matrix<double, 2, 1> z_hat = H * x_;
  const Eigen::Matrix<double, 2, 1> innovation = z - z_hat;

  Eigen::Matrix<double, 2, 2> S = H * P_ * H.transpose() + R;
  Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();

  x_ = x_ + K * innovation;
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  P_ = (I - K * H) * P_;

  const double speed = std::hypot(x_(2), x_(3));
  if (speed > params_.max_plausible_velocity) {
    const double scale = params_.max_plausible_velocity / speed;
    x_(2) *= scale;
    x_(3) *= scale;
  }
  const double clamped_speed = std::hypot(x_(2), x_(3));

  state_.stamp = stamp;
  state_.x = x_(0);
  state_.y = x_(1);
  state_.vx = x_(2);
  state_.vy = x_(3);
  state_.ax = dt > 0.0 ? (state_.vx - previous_vx) / dt : 0.0;
  state_.ay = dt > 0.0 ? (state_.vy - previous_vy) / dt : 0.0;
  state_.speed = clamped_speed;
  if (state_.speed > params_.min_speed_for_heading) {
    const double yaw = std::atan2(state_.vy, state_.vx);
    state_.yaw = yaw;

    double yaw_unwrapped = yaw;
    if (have_last_yaw_) {
      yaw_unwrapped =
        last_yaw_unwrapped_ + normalizeAngle(yaw - last_yaw_raw_);
    }
    have_last_yaw_ = true;
    last_yaw_raw_ = yaw;
    last_yaw_unwrapped_ = yaw_unwrapped;
    yaw_history_.emplace_back(stamp, yaw_unwrapped);

    while (!yaw_history_.empty() &&
      stamp - yaw_history_.front().first > params_.omega_window_time)
    {
      yaw_history_.pop_front();
    }

    if (yaw_history_.size() >= 3) {
      const double t0 = yaw_history_.front().first;
      const double time_span = yaw_history_.back().first - t0;
      if (time_span > 0.15) {
        double mean_t = 0.0;
        double mean_yaw = 0.0;
        for (const auto & sample : yaw_history_) {
          mean_t += sample.first - t0;
          mean_yaw += sample.second;
        }
        mean_t /= static_cast<double>(yaw_history_.size());
        mean_yaw /= static_cast<double>(yaw_history_.size());

        double covariance = 0.0;
        double variance = 0.0;
        for (const auto & sample : yaw_history_) {
          const double t = (sample.first - t0) - mean_t;
          const double y = sample.second - mean_yaw;
          covariance += t * y;
          variance += t * t;
        }

        if (variance > 1e-9) {
          double raw_omega = covariance / variance;
          raw_omega = clip(
            raw_omega, -params_.max_plausible_omega, params_.max_plausible_omega);
          if (std::abs(raw_omega) < params_.omega_deadband) {
            raw_omega = 0.0;
          }
          state_.omega =
            params_.omega_filter_alpha * raw_omega +
            (1.0 - params_.omega_filter_alpha) * state_.omega;
        }
      }
    }
  } else {
    yaw_history_.clear();
    have_last_yaw_ = false;
    state_.omega *= (1.0 - params_.omega_filter_alpha);
  }
}

void TargetStateEstimator::setYaw(double yaw)
{
  state_.yaw = normalizeAngle(yaw);
  have_last_yaw_ = false;
  yaw_history_.clear();
}

void TargetStateEstimator::updateVelocity(
  double stamp,
  double vx,
  double vy,
  double omega,
  bool has_omega)
{
  if (!state_.initialized) {
    return;
  }

  double speed = std::hypot(vx, vy);
  if (speed > params_.max_plausible_velocity) {
    const double scale = params_.max_plausible_velocity / speed;
    vx *= scale;
    vy *= scale;
    speed = params_.max_plausible_velocity;
  }

  x_(2) = vx;
  x_(3) = vy;
  state_.stamp = stamp;
  state_.vx = vx;
  state_.vy = vy;
  state_.speed = speed;
  if (speed > params_.min_speed_for_heading) {
    state_.yaw = std::atan2(vy, vx);
  }

  if (has_omega) {
    state_.omega = clip(
      omega, -params_.max_plausible_omega, params_.max_plausible_omega);
    if (std::abs(state_.omega) < params_.omega_deadband) {
      state_.omega = 0.0;
    }
    // 直接角速度测量比航向角回归
    // 更可信；从当前样本重新开始滑窗。
    yaw_history_.clear();
    have_last_yaw_ = false;
  }
}

}  // 命名空间 docking
