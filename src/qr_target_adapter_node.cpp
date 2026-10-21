#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace docking
{

class QrTargetAdapterNode : public rclcpp::Node
{
public:
  QrTargetAdapterNode()
  : Node("qr_target_adapter_node")
  {
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "odom");
    qr_tag_frame_ = this->declare_parameter<std::string>("qr_tag_frame", "qr_tag");
    target_base_frame_ =
      this->declare_parameter<std::string>("target_base_frame", "target_base");
    publish_twist_ = this->declare_parameter<bool>("publish_twist", false);
    velocity_filter_alpha_ =
      this->declare_parameter<double>("velocity_filter_alpha", 0.35);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/target/pose", 5);
    target_twist_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/target/twist", 5);

    qr_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/qr/tag_pose", rclcpp::QoS(5),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onQrPose(msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "QR adapter started: planning_frame=%s qr_tag_frame=%s target_base_frame=%s",
      planning_frame_.c_str(), qr_tag_frame_.c_str(), target_base_frame_.c_str());
  }

private:
  void onQrPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped tag_in_planning;
    try {
      tag_in_planning = tf_buffer_->transform(*msg, planning_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "QR tag transform failed: %s", ex.what());
      return;
    }

    tf2::Transform T_odom_tag;
    tf2::fromMsg(tag_in_planning.pose, T_odom_tag);

    tf2::Transform T_target_tag = tf2::Transform::getIdentity();
    try {
      const auto tag_to_target = tf_buffer_->lookupTransform(target_base_frame_, qr_tag_frame_, tf2::TimePointZero);
      tf2::fromMsg(tag_to_target.transform, T_target_tag);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "QR tag->target_base transform failed: %s", ex.what());
      return;
    }

    const tf2::Transform T_odom_target = T_odom_tag * T_target_tag.inverse();
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = tag_in_planning.header.stamp;
    target_pose.header.frame_id = planning_frame_;
    tf2::toMsg(T_odom_target, target_pose.pose);
    target_pose_pub_->publish(target_pose);

    if (publish_twist_) {
      publishTargetTwist(target_pose);
    }
  }

  void publishTargetTwist(const geometry_msgs::msg::PoseStamped & pose)
  {
    const double stamp = rclcpp::Time(pose.header.stamp).seconds();
    const double yaw = yawFromQuaternion(pose.pose.orientation);

    if (have_last_pose_) {
      const double dt = stamp - last_stamp_;
      if (dt > 1e-3) {
        const double raw_vx = (pose.pose.position.x - last_x_) / dt;
        const double raw_vy = (pose.pose.position.y - last_y_) / dt;
        const double raw_omega = normalizeAngle(yaw - last_yaw_) / dt;

        vx_ = filter(velocity_filter_alpha_, raw_vx, vx_);
        vy_ = filter(velocity_filter_alpha_, raw_vy, vy_);
        omega_ = filter(velocity_filter_alpha_, raw_omega, omega_);

        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = pose.header.stamp;
        twist.header.frame_id = planning_frame_;
        twist.twist.linear.x = vx_;
        twist.twist.linear.y = vy_;
        twist.twist.angular.z = omega_;
        target_twist_pub_->publish(twist);
      }
    }

    have_last_pose_ = true;
    last_stamp_ = stamp;
    last_x_ = pose.pose.position.x;
    last_y_ = pose.pose.position.y;
    last_yaw_ = yaw;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double filter(double alpha, double measurement, double previous)
  {
    return (1.0 - alpha) * previous + alpha * measurement;
  }

  std::string planning_frame_;
  std::string qr_tag_frame_;
  std::string target_base_frame_;
  bool publish_twist_{true};
  double velocity_filter_alpha_{0.35};

  bool have_last_pose_{false};
  double last_stamp_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  double vx_{0.0};
  double vy_{0.0};
  double omega_{0.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
};

}  // namespace docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking::QrTargetAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
