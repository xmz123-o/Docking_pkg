#include "docking_pkg/docking_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>

namespace docking
{

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

DockingPlannerNode::DockingPlannerNode()
: Node("docking_planner_node")
{
  planner_mode_ = this->declare_parameter<std::string>("planner_mode", "frenet");
  planning_frame_ = this->declare_parameter<std::string>("planning_frame", "ego_base");
  body_frame_planning_ =
    this->declare_parameter<bool>("body_frame_planning", true);
  target_warmup_cycles_ =
    this->declare_parameter<int>("target.warmup_cycles", 10);

  PositionProcessor::Params position_params;
  position_params.camera_x =
    this->declare_parameter<double>("position_processor.camera_x", 0.35);
  position_params.camera_y =
    this->declare_parameter<double>("position_processor.camera_y", 0.0);
  position_params.camera_yaw =
    this->declare_parameter<double>("position_processor.camera_yaw", 0.0);
  position_params.tag_x =
    this->declare_parameter<double>("position_processor.tag_x", -0.26);
  position_params.tag_y =
    this->declare_parameter<double>("position_processor.tag_y", 0.0);
  position_params.tag_yaw =
    this->declare_parameter<double>("position_processor.tag_yaw", 0.0);
  position_params.min_range =
    this->declare_parameter<double>("position_processor.min_range", 0.05);
  position_params.max_range =
    this->declare_parameter<double>("position_processor.max_range", 50.0);
  position_processor_ = PositionProcessor(position_params);

  EgoMotionCompensator::Params ego_motion_params;
  ego_motion_params.yaw_rate_filter_alpha =
    this->declare_parameter<double>("ego.yaw_rate_filter_alpha", 0.2);
  ego_motion_params.max_yaw_rate =
    this->declare_parameter<double>("ego.max_yaw_rate", 2.0);
  ego_motion_params.yaw_rate_deadband =
    this->declare_parameter<double>("ego.yaw_rate_deadband", 0.01);
  ego_compensator_ = EgoMotionCompensator(ego_motion_params);

  TargetVelocityFilter::Params filter_params;
  filter_params.alpha =
    this->declare_parameter<double>("target.velocity_filter_alpha", 0.25);
  filter_params.max_speed =
    this->declare_parameter<double>("target.velocity_max", 8.0);
  filter_params.min_speed_for_heading =
    this->declare_parameter<double>("target.velocity_min_speed_for_heading", 0.03);
  target_velocity_filter_ = TargetVelocityFilter(filter_params);
  const double desired_center_distance =
    this->declare_parameter<double>("docking.desired_center_distance", 0.52);
  control_period_ = this->declare_parameter<double>("control_period", 0.05);
  publish_markers_ = this->declare_parameter<bool>("publish_markers", true);

  TargetStateEstimator::Params estimator_params;
  estimator_params.measurement_noise_position =
    this->declare_parameter<double>("target.measurement_noise_position", 0.0001);
  estimator_params.process_noise_position =
    this->declare_parameter<double>("target.process_noise_position", 0.02);
  estimator_params.process_noise_velocity =
    this->declare_parameter<double>("target.process_noise_velocity", 0.05);
  estimator_params.max_plausible_velocity =
    this->declare_parameter<double>("target.max_plausible_velocity", 3.0);
  estimator_params.min_speed_for_heading =
    this->declare_parameter<double>("target.min_speed_for_heading", 0.08);
  estimator_params.omega_window_time =
    this->declare_parameter<double>("target.omega_window_time", 2.5);
  estimator_params.omega_filter_alpha =
    this->declare_parameter<double>("target.omega_filter_alpha", 0.08);
  estimator_params.omega_deadband =
    this->declare_parameter<double>("target.omega_deadband", 0.04);
  estimator_params.max_plausible_omega =
    this->declare_parameter<double>("target.max_plausible_omega", 0.8);
  estimator_params.use_measured_orientation =
    this->declare_parameter<bool>("target.use_measured_orientation", true);
  estimator_ = TargetStateEstimator(estimator_params);

  TargetPredictor::Params predictor_params;
  predictor_params.prediction_dt =
    this->declare_parameter<double>("prediction.dt", 0.1);
  predictor_params.prediction_horizon =
    this->declare_parameter<double>("prediction.horizon", 10.0);
  predictor_params.min_speed_for_ctrv =
    this->declare_parameter<double>("prediction.min_speed_for_ctrv", 0.02);
  predictor_params.max_abs_omega =
    this->declare_parameter<double>("prediction.max_abs_omega", 0.8);
  predictor_params.omega_deadband =
    this->declare_parameter<double>("prediction.omega_deadband", 0.04);
  predictor_ = TargetPredictor(predictor_params);

  ReferenceLineGenerator::Params reference_params;
  reference_params.target_length =
    this->declare_parameter<double>("vehicle.target_length", 0.52);
  reference_params.ego_length =
    this->declare_parameter<double>("vehicle.ego_length", 0.52);
  reference_params.cone_bus_length =
    this->declare_parameter<double>("vehicle.cone_bus_length", 0.08);
  reference_params.docking_depth =
    this->declare_parameter<double>("vehicle.docking_depth", 0.05);
  reference_params.sample_count =
    this->declare_parameter<int>("planning.reference_sample_count", 80);
  reference_params.derivative_scale =
    this->declare_parameter<double>("planning.reference_derivative_scale", 1.0);
  reference_params.max_ego_speed =
    this->declare_parameter<double>("planning.max_ego_speed", 1.0);
  reference_params.max_ego_accel =
    this->declare_parameter<double>("planning.max_ego_accel", 0.8);
  reference_params.max_ego_decel =
    this->declare_parameter<double>("planning.max_ego_decel", 1.0);
  reference_params.min_docking_time =
    this->declare_parameter<double>("planning.min_docking_time", 0.2);
  reference_params.max_docking_time =
    this->declare_parameter<double>("planning.max_docking_time", 10.0);
  reference_params.prealign_distance =
    this->declare_parameter<double>("planning.prealign_distance", 0.2);
  reference_params.prealign_activation_distance =
    this->declare_parameter<double>("planning.prealign_activation_distance", 1.2);
  reference_params.docking_goal_lead =
    this->declare_parameter<double>("planning.docking_goal_lead", 0.08);
  reference_params.desired_center_distance = desired_center_distance;
  reference_params.path_length_margin =
    this->declare_parameter<double>("planning.path_length_margin", 1.05);
  reference_generator_ = ReferenceLineGenerator(reference_params);

  LocalPlanner::Params local_params;
  local_params.mode = planner_mode_ == "frenet"
    ? LocalPlanner::Mode::kFrenet
    : LocalPlanner::Mode::kCartesian;
  local_params.sample_count =
    this->declare_parameter<int>("planning.local_sample_count", 80);
  local_planner_ = LocalPlanner(local_params);

  SpeedPlanner::Params speed_params;
  speed_params.max_accel =
    this->declare_parameter<double>("speed.max_accel", 0.8);
  speed_params.max_decel =
    this->declare_parameter<double>("speed.max_decel", 1.0);
  speed_params.integration_dt =
    this->declare_parameter<double>("speed.integration_dt", 0.02);
  speed_planner_ = SpeedPlanner(speed_params);

  DockingStateMachine::Params state_params;
  state_params.tracking_distance =
    desired_center_distance +
    this->declare_parameter<double>("docking.tracking_distance_extra", 1.70);
  state_params.approach_distance =
    desired_center_distance +
    this->declare_parameter<double>("docking.approach_distance_extra", 0.70);
  state_params.align_distance =
    desired_center_distance +
    this->declare_parameter<double>("docking.align_distance_extra", 0.32);
  state_params.final_dock_distance =
    desired_center_distance +
    this->declare_parameter<double>("docking.final_dock_distance_extra", 0.12);
  state_params.docked_distance =
    desired_center_distance +
    this->declare_parameter<double>("docking.docked_distance_extra", 0.02);
  state_params.docked_speed =
    this->declare_parameter<double>("docking.docked_speed", 0.08);
  state_params.final_heading_error =
    this->declare_parameter<double>("docking.final_heading_error", 0.35);
  state_params.docked_heading_error =
    this->declare_parameter<double>("docking.docked_heading_error", 0.25);
  state_params.final_lateral_error =
    this->declare_parameter<double>("docking.final_lateral_error", 0.20);
  state_params.docked_lateral_error =
    this->declare_parameter<double>("docking.docked_lateral_error", 0.15);
  state_params.tracking_speed =
    this->declare_parameter<double>("docking.tracking_speed", 1.0);
  state_params.approach_speed =
    this->declare_parameter<double>("docking.approach_speed", 0.6);
  state_params.align_speed =
    this->declare_parameter<double>("docking.align_speed", 0.35);
  state_params.final_speed =
    this->declare_parameter<double>("docking.final_speed", 0.12);
  state_params.docked_hold_speed =
    this->declare_parameter<double>("docking.docked_hold_speed", 0.0);
  state_params.desired_center_distance = desired_center_distance;
  state_params.docked_longitudinal_error =
    this->declare_parameter<double>("docking.docked_longitudinal_error", 0.02);
  state_params.final_closure_speed =
    this->declare_parameter<double>("docking.final_closure_speed", 0.04);
  state_params.docked_release_margin =
    this->declare_parameter<double>("docking.docked_release_margin", 0.30);
  state_params.max_ego_speed =
    this->declare_parameter<double>("docking.max_ego_speed", 1.0);
  state_params.catch_up_gain =
    this->declare_parameter<double>("docking.catch_up_gain", 0.8);
  state_params.max_closure_speed =
    this->declare_parameter<double>("docking.max_closure_speed", 0.6);
  state_params.min_closure_speed =
    this->declare_parameter<double>("docking.min_closure_speed", 0.03);
  state_params.body_frame_mode = body_frame_planning_;
  state_machine_ = DockingStateMachine(state_params);

  DifferentialController::Params controller_params;
  controller_params.wheel_radius =
    this->declare_parameter<double>("vehicle.wheel_radius", 0.254);
  controller_params.wheel_track =
    this->declare_parameter<double>("vehicle.wheel_track", 0.60);
  controller_params.max_linear =
    this->declare_parameter<double>("control.max_linear", 1.0);
  controller_params.max_angular =
    this->declare_parameter<double>("control.max_angular", 1.2);
  controller_params.max_wheel_rad_s =
    this->declare_parameter<double>("control.max_wheel_rad_s", 8.0);
  controller_params.lookahead_distance =
    this->declare_parameter<double>("control.lookahead_distance", 0.12);
  controller_params.lookahead_time =
    this->declare_parameter<double>("control.lookahead_time", 0.30);
  controller_params.max_lookahead_distance =
    this->declare_parameter<double>("control.max_lookahead_distance", 0.30);
  controller_params.path_heading_gain =
    this->declare_parameter<double>("control.path_heading_gain", 2.5);
  controller_params.path_feedforward_gain =
    this->declare_parameter<double>("control.path_feedforward_gain", 0.8);
  controller_ = DifferentialController(controller_params);

  ego_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ego/pose", rclcpp::QoS(5),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onEgoPose(msg); });
  target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target/pose", rclcpp::QoS(5),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onTargetPose(msg); });
  ego_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/ego/twist", rclcpp::QoS(5),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { onEgoTwist(msg); });
  target_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/target/twist", rclcpp::QoS(5),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { onTargetTwist(msg); });
  qr_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/qr/tag_position", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { onQrPosition(msg); });
  ego_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ego/yaw", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64::SharedPtr msg) { onEgoYaw(msg); });

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 5);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/markers", 5);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
  wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_cmd", 5);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/docking/status", 5);

  const int period_ms = static_cast<int>(std::max(10.0, control_period_ * 1000.0));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms), [this]() { onTimer(); });

  last_cycle_time_ = this->now();
  last_status_log_time_ = this->now();
  previous_ego_ = ego_pose_;
  RCLCPP_INFO(this->get_logger(), "Docking planner started, mode=%s", planner_mode_.c_str());
}

void DockingPlannerNode::onEgoPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (body_frame_planning_) {
    ego_pose_ = Pose2D{0.0, 0.0, 0.0};
    ego_stamp_ = msg->header.stamp;
    have_ego_ = true;
    return;
  }
  if (!have_ego_) {
    previous_ego_ = toPose2D(*msg);
  } else {
    const double dt = (rclcpp::Time(msg->header.stamp) - ego_stamp_).seconds();
    if (dt > 1e-3) {
      const Pose2D current = toPose2D(*msg);
      ego_speed_ = distance(previous_ego_.x, previous_ego_.y, current.x, current.y) / dt;
      previous_ego_ = current;
    }
  }
  ego_pose_ = toPose2D(*msg);
  ego_stamp_ = msg->header.stamp;
  have_ego_ = true;
}

void DockingPlannerNode::onTargetPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (body_frame_planning_) {
    return;
  }
  target_pose_ = toPose2D(*msg);
  target_stamp_ = msg->header.stamp;
  have_target_ = true;
  estimator_.update(target_stamp_.seconds(), target_pose_.x, target_pose_.y);
  if (estimator_.useMeasuredOrientation()) {
    estimator_.setYaw(target_pose_.yaw);
  }
}

void DockingPlannerNode::onQrPosition(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  qr_position_.valid = true;
  qr_position_.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  qr_position_.x = msg->point.x;
  qr_position_.y = msg->point.y;
  qr_position_.z = msg->point.z;
  qr_position_updated_ = true;
}

void DockingPlannerNode::onEgoYaw(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  ego_yaw_ = msg->data;
  ego_yaw_stamp_ = this->now();
  have_ego_yaw_ = true;
}

void DockingPlannerNode::onEgoTwist(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  ego_twist_vx_ = msg->twist.linear.x;
  ego_twist_vy_ = msg->twist.linear.y;
  ego_twist_omega_ = msg->twist.angular.z;
  ego_twist_stamp_ = msg->header.stamp;
  have_ego_twist_ = true;
}

void DockingPlannerNode::onTargetTwist(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  target_twist_vx_ = msg->twist.linear.x;
  target_twist_vy_ = msg->twist.linear.y;
  target_twist_omega_ = msg->twist.angular.z;
  target_twist_stamp_ = msg->header.stamp;
  have_target_twist_ = true;
}

Pose2D DockingPlannerNode::toPose2D(
  const geometry_msgs::msg::PoseStamped & msg) const
{
  Pose2D pose;
  pose.x = msg.pose.position.x;
  pose.y = msg.pose.position.y;
  pose.yaw = yawFromQuaternion(msg.pose.orientation);
  return pose;
}

void DockingPlannerNode::onTimer()
{
  const rclcpp::Time now = this->now();
  const double cycle_dt = (now - last_cycle_time_).seconds();
  last_cycle_time_ = now;

  // 车体系模式：先用二维码位置更新相对状态估计。
  if (body_frame_planning_ && qr_position_updated_) {
    const double qr_ego_speed = have_ego_twist_
      ? std::hypot(ego_twist_vx_, ego_twist_vy_)
      : 0.0;
    if (have_ego_yaw_) {
      ego_compensator_.setEgoYaw(ego_yaw_stamp_.seconds(), ego_yaw_);
    }
    const double ego_omega = have_ego_yaw_
      ? ego_compensator_.egoOmega()
      : ego_twist_omega_;
    const double yaw_estimate = target_velocity_filter_.state().initialized
      ? target_velocity_filter_.state().yaw
      : 0.0;

    Pose2D target_in_ego;
    if (position_processor_.process(qr_position_, yaw_estimate, &target_in_ego)) {
      ego_compensator_.setEgoState(
        qr_position_.stamp, qr_ego_speed, ego_omega);
      double vx_relative = 0.0;
      double vy_relative = 0.0;
      ego_compensator_.compensate(
        target_in_ego, qr_position_.stamp, &vx_relative, &vy_relative);
      target_velocity_filter_.update(
        qr_position_.stamp, target_in_ego.x, target_in_ego.y,
        vx_relative, vy_relative, qr_ego_speed, ego_omega);
      target_valid_count_ = std::min(
        target_warmup_cycles_, target_valid_count_ + 1);
      have_target_ = true;
    }
    qr_position_updated_ = false;
  }

  if (have_target_ && target_valid_count_ < target_warmup_cycles_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    std_msgs::msg::Float64MultiArray wheels;
    wheels.data = {0.0, 0.0};
    wheel_pub_->publish(wheels);
    std_msgs::msg::String status;
    status.data = "WARMUP";
    status_pub_->publish(status);
    return;
  }

  const bool target_ready = body_frame_planning_
    ? target_velocity_filter_.state().initialized
    : estimator_.state().initialized;
  if (!have_ego_ || !have_target_ || !target_ready) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    std_msgs::msg::Float64MultiArray wheels;
    wheels.data = {0.0, 0.0};
    wheel_pub_->publish(wheels);
    std_msgs::msg::String status;
    status.data = "WAITING_FOR_TARGET";
    status_pub_->publish(status);
    return;
  }

  if (body_frame_planning_) {
    ego_pose_ = Pose2D{0.0, 0.0, 0.0};
  }

  if (!body_frame_planning_ && have_target_twist_) {
    estimator_.updateVelocity(
      target_twist_stamp_.seconds(),
      target_twist_vx_,
      target_twist_vy_,
      target_twist_omega_,
      true);
  }

  const double ego_speed = have_ego_twist_
    ? std::hypot(ego_twist_vx_, ego_twist_vy_)
    : ego_speed_;
  const double now_seconds = now.seconds();

  TargetState target;
  Trajectory predicted;
  if (body_frame_planning_) {
    target = target_velocity_filter_.state();
    predicted = predictor_.predictRelative(target, now_seconds);
  } else {
    const TargetState target_measured = estimator_.state();
    const double target_elapsed = target_measured.stamp > 0.0
      ? std::max(0.0, now_seconds - target_measured.stamp)
      : 0.0;
    target = predictor_.propagate(target_measured, target_elapsed);
    predicted = predictor_.predict(target, now_seconds);
  }

  const DockingState docking_state = state_machine_.update(
    ego_pose_, target, ego_speed);

  Path local_path;
  Trajectory trajectory;
  ReferenceLine reference;
  WheelCommand command;

  if (body_frame_planning_) {
    // 车体系第一版：生成从自车原点到当前对接点的稳定直线参考路径。
    const double desired = reference_generator_.desired_center_distance();
    const double goal_x = target.x - desired * std::cos(target.yaw);
    const double goal_y = target.y - desired * std::sin(target.yaw);
    const double goal_yaw = std::atan2(goal_y, goal_x);

    const int samples = 40;
    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    for (int i = 0; i <= samples; ++i) {
      const double u = static_cast<double>(i) / static_cast<double>(samples);
      PathPoint point;
      point.x = goal_x * u;
      point.y = goal_y * u;
      point.yaw = goal_yaw;
      if (i > 0) {
        s += distance(prev_x, prev_y, point.x, point.y);
      }
      point.s = s;
      point.kappa = 0.0;
      local_path.push_back(point);
      prev_x = point.x;
      prev_y = point.y;
    }

    double t = 0.0;
    for (const auto & point : local_path) {
      TrajectoryPoint tp;
      tp.t = t;
      tp.x = point.x;
      tp.y = point.y;
      tp.yaw = point.yaw;
      tp.v = std::max(ego_speed, 0.0);
      tp.a = 0.0;
      tp.omega = 0.0;
      tp.kappa = 0.0;
      tp.s = point.s;
      trajectory.push_back(tp);
      t += 0.05;
    }

    reference.path = local_path;
    reference.docking_goal.valid = true;
    reference.docking_goal.time = now_seconds + std::max(
      0.2, local_path.empty() ? 0.0 : local_path.back().s /
      std::max(0.2, ego_speed));
    reference.docking_goal.pose = Pose2D{goal_x, goal_y, goal_yaw};
    reference.docking_goal.speed = target.speed;
    reference.docking_goal.omega = target.omega;

    command = controller_.computeBodyRelative(ego_speed, target, desired);
  } else {
    reference = reference_generator_.generate(ego_pose_, ego_speed, predicted);
    local_path = local_planner_.plan(ego_pose_, reference.path);
    const double target_time = reference.docking_goal.valid
      ? reference.docking_goal.time - now_seconds
      : -1.0;
    const double final_speed = reference.docking_goal.valid
      ? reference.docking_goal.speed
      : docking_state.final_speed;
    trajectory = speed_planner_.plan(
      local_path, ego_speed, docking_state.cruise_speed, final_speed, target_time);
    command = controller_.compute(
      ego_pose_, trajectory, docking_state.cruise_speed);
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = command.linear;
  cmd_vel.angular.z = command.angular;
  cmd_vel_pub_->publish(cmd_vel);

  std_msgs::msg::Float64MultiArray wheel_msg;
  wheel_msg.data = {command.left_rad_s, command.right_rad_s};
  wheel_pub_->publish(wheel_msg);

  publishTrajectory(now, local_path, trajectory, predicted, reference, target);
  publishStatus(docking_state, target, reference, trajectory, now_seconds);

  if ((now - last_status_log_time_).seconds() > 2.0) {
    const double docking_time = reference.docking_goal.valid
      ? reference.docking_goal.time - now_seconds
      : -1.0;
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] target_v=(%.2f,%.2f) target_speed=%.2f ego_speed=%.2f closure=%.2f dist=%.2f lat_err=%.2f yaw_err=%.2f td=%.2f arrival=%.2f a=%.2f wheel=(%.2f, %.2f)",
      DockingStateMachine::phaseName(docking_state.phase).c_str(),
      target.vx, target.vy, target.speed, ego_speed,
      docking_state.closing_speed,
      docking_state.center_distance,
      docking_state.lateral_error,
      docking_state.heading_error,
      docking_time,
      trajectory.empty() ? 0.0 : trajectory.back().t,
      trajectory.empty() ? 0.0 : trajectory.front().a,
      command.left_rad_s, command.right_rad_s);
    last_status_log_time_ = now;
  }

  (void)cycle_dt;
}

void DockingPlannerNode::publishTrajectory(
  const rclcpp::Time & stamp,
  const Path & path,
  const Trajectory & trajectory,
  const Trajectory & predicted_target,
  const ReferenceLine & reference,
  const TargetState & target)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = stamp;
  path_msg.header.frame_id = planning_frame_;
  for (const auto & point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation = quaternionFromYaw(point.yaw);
    path_msg.poses.push_back(pose);
  }
  path_pub_->publish(path_msg);

  if (!publish_markers_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker line;
  line.header = path_msg.header;
  line.ns = "trajectory";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = 0.02;
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;
  line.color.a = 0.9;
  for (const auto & point : trajectory) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.0;
    line.points.push_back(p);
  }
  markers.markers.push_back(line);

  if (!trajectory.empty()) {
    visualization_msgs::msg::Marker goal;
    goal.header = path_msg.header;
    goal.ns = "goal";
    goal.id = 1;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.scale.x = 0.08;
    goal.scale.y = 0.08;
    goal.scale.z = 0.08;
    goal.color.r = 1.0;
    goal.color.g = 0.5;
    goal.color.b = 0.0;
    goal.color.a = 1.0;
    const double docking_distance = reference_generator_.desired_center_distance();
    goal.pose.position.x = target.x - std::cos(target.yaw) * docking_distance;
    goal.pose.position.y = target.y - std::sin(target.yaw) * docking_distance;
    goal.pose.orientation = quaternionFromYaw(target.yaw);
    markers.markers.push_back(goal);
  }

  if (!predicted_target.empty()) {
    visualization_msgs::msg::Marker prediction;
    prediction.header = path_msg.header;
    prediction.ns = "target_prediction";
    prediction.id = 2;
    prediction.type = visualization_msgs::msg::Marker::LINE_STRIP;
    prediction.scale.x = 0.015;
    prediction.color.r = 0.1;
    prediction.color.g = 0.75;
    prediction.color.b = 1.0;
    prediction.color.a = 0.85;
    for (const auto & point : predicted_target) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      prediction.points.push_back(p);
    }
    markers.markers.push_back(prediction);
  }

  if (reference.docking_goal.valid) {
    const double docking_distance = reference_generator_.desired_center_distance();
    visualization_msgs::msg::Marker docking_goal;
    docking_goal.header = path_msg.header;
    docking_goal.ns = "docking_goal";
    docking_goal.id = 3;
    docking_goal.type = visualization_msgs::msg::Marker::SPHERE;
    docking_goal.scale.x = 0.06;
    docking_goal.scale.y = 0.06;
    docking_goal.scale.z = 0.06;
    docking_goal.color.r = 1.0;
    docking_goal.color.g = 0.9;
    docking_goal.color.b = 0.0;
    docking_goal.color.a = 1.0;
    docking_goal.pose.position.x = target.x - std::cos(target.yaw) * docking_distance;
    docking_goal.pose.position.y = target.y - std::sin(target.yaw) * docking_distance;
    docking_goal.pose.orientation = quaternionFromYaw(target.yaw);
    markers.markers.push_back(docking_goal);
  }
  marker_pub_->publish(markers);
}

void DockingPlannerNode::publishStatus(
  const DockingState & state,
  const TargetState & target,
  const ReferenceLine & reference,
  const Trajectory & trajectory,
  double now_seconds)
{
  std_msgs::msg::String msg;
  std::ostringstream oss;
  const double docking_time = reference.docking_goal.valid
    ? reference.docking_goal.time - now_seconds
    : -1.0;
  const double arrival_time = trajectory.empty() ? 0.0 : trajectory.back().t;
  oss << DockingStateMachine::phaseName(state.phase)
      << " dist=" << state.center_distance
      << " lon_err=" << state.longitudinal_error
      << " target_speed=" << target.speed
      << " cruise=" << state.cruise_speed
      << " closing_speed=" << state.closing_speed
      << " lat_err=" << state.lateral_error
      << " yaw_err=" << state.heading_error
      << " cruise=" << state.cruise_speed
      << " td=" << docking_time
      << " arrival_t=" << arrival_time
      << " time_err=" << (docking_time >= 0.0 ? arrival_time - docking_time : 0.0)
      << " a_cmd=" << (trajectory.empty() ? 0.0 : trajectory.front().a);
  msg.data = oss.str();
  status_pub_->publish(msg);
}

}  // namespace docking
