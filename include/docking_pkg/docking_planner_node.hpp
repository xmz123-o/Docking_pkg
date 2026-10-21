#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "docking_pkg/differential_controller.hpp"
#include "docking_pkg/docking_state_machine.hpp"
#include "docking_pkg/local_planner.hpp"
#include "docking_pkg/reference_line_generator.hpp"
#include "docking_pkg/relative_state_estimator.hpp"
#include "docking_pkg/speed_planner.hpp"
#include "docking_pkg/target_predictor.hpp"
#include "docking_pkg/target_state_estimator.hpp"
#include "docking_pkg/types.hpp"

namespace docking
{

class DockingPlannerNode : public rclcpp::Node
{
public:
  DockingPlannerNode();

private:
  void onEgoPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onQrPosition(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onEgoTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onEgoYaw(const std_msgs::msg::Float64::SharedPtr msg);
  void onTargetTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onTimer();

  Pose2D toPose2D(const geometry_msgs::msg::PoseStamped & msg) const;
  void publishTrajectory(
    const rclcpp::Time & stamp,
    const Path & path,
    const Trajectory & trajectory,
    const Trajectory & predicted_target,
    const ReferenceLine & reference,
    const TargetState & target);
  void publishStatus(
    const DockingState & state,
    const TargetState & target,
    const ReferenceLine & reference,
    const Trajectory & trajectory,
    double now_seconds);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ego_twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_yaw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr qr_position_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_ego_{false};
  bool have_target_{false};
  Pose2D ego_pose_;
  Pose2D target_pose_;
  rclcpp::Time ego_stamp_;
  rclcpp::Time target_stamp_;
  rclcpp::Time last_cycle_time_;
  rclcpp::Time last_status_log_time_;
  double ego_speed_{0.0};
  bool have_ego_twist_{false};
  bool have_target_twist_{false};
  double ego_twist_vx_{0.0};
  double ego_twist_vy_{0.0};
  double ego_twist_omega_{0.0};
  bool have_ego_yaw_{false};
  double ego_yaw_{0.0};
  rclcpp::Time ego_yaw_stamp_;
  double target_twist_vx_{0.0};
  double target_twist_vy_{0.0};
  double target_twist_omega_{0.0};
  rclcpp::Time ego_twist_stamp_;
  rclcpp::Time target_twist_stamp_;
  Pose2D previous_ego_;

  TargetStateEstimator estimator_;
  PositionProcessor position_processor_;
  EgoMotionCompensator ego_compensator_;
  TargetVelocityFilter target_velocity_filter_;
  QrPosition qr_position_;
  bool qr_position_updated_{false};
  TargetPredictor predictor_;
  ReferenceLineGenerator reference_generator_;
  LocalPlanner local_planner_;
  SpeedPlanner speed_planner_;
  DockingStateMachine state_machine_;
  DifferentialController controller_;

  std::string planner_mode_;
  std::string planning_frame_;
  bool body_frame_planning_{true};
  int target_warmup_cycles_{10};
  int target_valid_count_{0};
  double control_period_{0.05};
  bool publish_markers_{true};
};

}  // 命名空间 docking
