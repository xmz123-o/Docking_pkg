#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "docking_pkg/types.hpp"

namespace docking
{

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  return q;
}

void setPose(geometry_msgs::msg::Pose & pose,double x, double y, double z,double yaw)
{
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = quaternionFromYaw(yaw);
}

visualization_msgs::msg::Marker makeBodyMarker(
  const std::string & ns,
  int id,
  const Pose2D & pose,
  double length,
  double width,
  double height,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  setPose(marker.pose, pose.x, pose.y, height / 2.0, pose.yaw);
  marker.scale.x = length;
  marker.scale.y = width;
  marker.scale.z = height;
  marker.color = color;
  return marker;
}

visualization_msgs::msg::Marker makeInterfaceMarker(
  const std::string & ns,
  int id,
  const Pose2D & pose,
  bool front,
  double body_length,
  double cone_bus,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  const double dir = front ? 1.0 : -1.0;
  const double offset = body_length / 2.0 + cone_bus / 2.0;
  setPose(
    marker.pose,
    pose.x + dir * std::cos(pose.yaw) * offset,
    pose.y + dir * std::sin(pose.yaw) * offset,
    0.10,
    pose.yaw);
  marker.scale.x = cone_bus;
  marker.scale.y = 0.12;
  marker.scale.z = 0.12;
  marker.color = color;
  return marker;
}

visualization_msgs::msg::Marker makeArrowMarker(
  const std::string & ns,
  int id,
  const Pose2D & pose,
  double length,
  double scale,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  setPose(marker.pose, pose.x, pose.y, 0.45, pose.yaw);
  marker.scale.x = length;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = color;
  return marker;
}

visualization_msgs::msg::Marker makeTextMarker(
  const std::string & ns,
  int id,
  const Pose2D & pose,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  setPose(marker.pose, pose.x, pose.y, 0.65, 0.0);
  marker.text = text;
  marker.scale.z = 0.14;
  marker.color = color;
  return marker;
}

std_msgs::msg::ColorRGBA makeColor(
  float r, float g, float b, float a = 1.0f)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}


tf2::Transform transformFromPose(const Pose2D & pose)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  return tf2::Transform(q, tf2::Vector3(pose.x, pose.y, 0.0));
}

geometry_msgs::msg::TransformStamped toTransformStamped(
  const std::string & parent,
  const std::string & child,
  const tf2::Transform & transform,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  const auto & origin = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  msg.transform.translation.x = origin.x();
  msg.transform.translation.y = origin.y();
  msg.transform.translation.z = origin.z();
  msg.transform.rotation.x = rotation.x();
  msg.transform.rotation.y = rotation.y();
  msg.transform.rotation.z = rotation.z();
  msg.transform.rotation.w = rotation.w();
  return msg;
}

}  // 命名空间

class DemoSimNode : public rclcpp::Node
{
public:
  DemoSimNode()
  : Node("demo_sim_node"),
    random_engine_(std::random_device{}())
  {
    const double ego_x = this->declare_parameter<double>("sim.ego_x", 0.0);
    const double ego_y = this->declare_parameter<double>("sim.ego_y", 0.0);
    const double ego_yaw = this->declare_parameter<double>("sim.ego_yaw", 0.0);
    const double target_x = this->declare_parameter<double>("sim.target_x", 2.5);
    const double target_y = this->declare_parameter<double>("sim.target_y", 0.5);
    const double target_yaw = this->declare_parameter<double>("sim.target_yaw", 0.0);
    target_speed_ = this->declare_parameter<double>("sim.target_speed", 0.35);
    target_yaw_rate_ = this->declare_parameter<double>("sim.target_yaw_rate", 0.08);
    wheel_radius_ = this->declare_parameter<double>("sim.wheel_radius", 0.254);
    wheel_track_ = this->declare_parameter<double>("sim.wheel_track", 0.60);
    publish_hz_ = this->declare_parameter<double>("sim.publish_hz", 50.0);
    measurement_noise_std_ =
      this->declare_parameter<double>("sim.measurement_noise_std", 0.008);
    velocity_noise_std_ =
      this->declare_parameter<double>("sim.velocity_noise_std", 0.01);
    body_length_ = this->declare_parameter<double>("sim.body_length", 0.52);
    body_width_ = this->declare_parameter<double>("sim.body_width", 0.45);
    body_height_ = this->declare_parameter<double>("sim.body_height", 0.40);
    cone_bus_ = this->declare_parameter<double>("sim.cone_bus", 0.08);
    qr_pose_noise_std_ =
      this->declare_parameter<double>("sim.qr_pose_noise_std", 0.005);
    camera_x_ = this->declare_parameter<double>("sim.camera_x", 0.35);
    camera_y_ = this->declare_parameter<double>("sim.camera_y", 0.0);
    camera_z_ = this->declare_parameter<double>("sim.camera_z", 0.20);
    camera_yaw_ = this->declare_parameter<double>("sim.camera_yaw", 0.0);
    tag_x_ = this->declare_parameter<double>("sim.tag_x", -0.26);
    tag_y_ = this->declare_parameter<double>("sim.tag_y", 0.0);
    tag_z_ = this->declare_parameter<double>("sim.tag_z", 0.05);
    tag_yaw_ = this->declare_parameter<double>("sim.tag_yaw", 0.0);

    ego_true_ = Pose2D{ego_x, ego_y, ego_yaw};
    target_true_ = Pose2D{target_x, target_y, target_yaw};

    tf2::Quaternion camera_rotation;
    camera_rotation.setRPY(0.0, 0.0, camera_yaw_);
    T_ego_camera_ = tf2::Transform(
      camera_rotation, tf2::Vector3(camera_x_, camera_y_, camera_z_));

    tf2::Quaternion tag_rotation;
    tag_rotation.setRPY(0.0, 0.0, tag_yaw_);
    T_target_tag_ = tf2::Transform(
      tag_rotation, tf2::Vector3(tag_x_, tag_y_, tag_z_));

    ego_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ego/pose", 5);
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/target_ground_truth/pose", 5);
    ego_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ego/twist", 5);
    target_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/target_ground_truth/twist", 5);
    qr_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/qr/tag_pose", 5);
    qr_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/qr/tag_position", 10);
    ego_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ego/yaw", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/simulation/markers", 5);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransforms();

    wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/wheel_cmd", rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
          left_wheel_ = msg->data[0];
          right_wheel_ = msg->data[1];
        }
      });

    const int period_ms = static_cast<int>(std::max(5.0, 1000.0 / publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms), [this]() { onTimer(); });
  }

private:
  void onTimer()
  {
    const double dt = 1.0 / std::max(5.0, publish_hz_);
    const rclcpp::Time stamp = this->now();

    target_true_.yaw += target_yaw_rate_ * dt;
    target_true_.yaw = normalizeAngle(target_true_.yaw);
    target_true_.x += target_speed_ * std::cos(target_true_.yaw) * dt;
    target_true_.y += target_speed_ * std::sin(target_true_.yaw) * dt;

    const double left_linear = left_wheel_ * wheel_radius_;
    const double right_linear = right_wheel_ * wheel_radius_;
    const double linear = 0.5 * (left_linear + right_linear);
    const double angular = (right_linear - left_linear) / wheel_track_;
    ego_true_.x += linear * std::cos(ego_true_.yaw) * dt;
    ego_true_.y += linear * std::sin(ego_true_.yaw) * dt;
    ego_true_.yaw += angular * dt;
    ego_true_.yaw = normalizeAngle(ego_true_.yaw);

    // 模拟定位：规划器只接收带少量噪声的测量。
    // 目标测量刻意不提供有意义的朝向。
    const Pose2D ego_measurement = addMeasurementNoise(ego_true_, true);
    Pose2D target_measurement = addMeasurementNoise(target_true_, false);
    target_measurement.yaw = 0.0;

    publishPose(ego_pub_, stamp, ego_measurement);
    publishPose(target_pub_, stamp, target_measurement);
    publishTwist(
      ego_twist_pub_,
      stamp,
      linear * std::cos(ego_true_.yaw),
      linear * std::sin(ego_true_.yaw),
      angular);
    publishTwist(
      target_twist_pub_,
      stamp,
      target_speed_ * std::cos(target_true_.yaw),
      target_speed_ * std::sin(target_true_.yaw),
      target_yaw_rate_);
    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = ego_true_.yaw;
    ego_yaw_pub_->publish(yaw_msg);

    publishTransforms(stamp);
    publishQrPose(stamp);
    publishMarkers(stamp);
  }

  Pose2D addMeasurementNoise(const Pose2D & pose, bool include_yaw)
  {
    std::normal_distribution<double> noise(0.0, measurement_noise_std_);
    Pose2D measured = pose;
    measured.x += noise(random_engine_);
    measured.y += noise(random_engine_);
    if (include_yaw) {
      std::normal_distribution<double> yaw_noise(0.0, 0.004);
      measured.yaw = normalizeAngle(measured.yaw + yaw_noise(random_engine_));
    }
    return measured;
  }

  void publishPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher,
    const rclcpp::Time & stamp,
    const Pose2D & pose)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = quaternionFromYaw(pose.yaw);
    publisher->publish(msg);
  }

  void publishTwist(
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher,
    const rclcpp::Time & stamp,
    double vx,
    double vy,
    double omega)
  {
    std::normal_distribution<double> noise(0.0, velocity_noise_std_);
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.twist.linear.x = vx + noise(random_engine_);
    msg.twist.linear.y = vy + noise(random_engine_);
    msg.twist.angular.z = omega + 0.5 * noise(random_engine_);
    publisher->publish(msg);
  }

  void publishMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;

    // 绘制真值车体，便于观察对接过程。
    markers.markers.push_back(makeBodyMarker(
      "ego", 0, ego_true_, body_length_, body_width_, body_height_,
      makeColor(0.15f, 0.85f, 0.35f, 0.9f)));
    markers.markers.push_back(makeBodyMarker(
      "target", 0, target_true_, body_length_, body_width_, body_height_,
      makeColor(0.90f, 0.25f, 0.20f, 0.9f)));

    markers.markers.push_back(makeInterfaceMarker(
      "ego", 1, ego_true_, true, body_length_, cone_bus_,
      makeColor(1.0f, 0.85f, 0.1f, 1.0f)));
    markers.markers.push_back(makeInterfaceMarker(
      "target", 1, target_true_, false, body_length_, cone_bus_,
      makeColor(0.35f, 0.35f, 0.40f, 1.0f)));

    markers.markers.push_back(makeArrowMarker(
      "ego", 2, ego_true_, 0.35, 0.035,
      makeColor(0.0f, 0.0f, 0.0f, 0.6f)));
    markers.markers.push_back(makeArrowMarker(
      "target", 2, target_true_, 0.35, 0.035,
      makeColor(1.0f, 1.0f, 1.0f, 0.8f)));

    markers.markers.push_back(makeTextMarker(
      "ego", 3, ego_true_, "Ego", makeColor(0.1f, 0.7f, 0.3f)));
    markers.markers.push_back(makeTextMarker(
      "target", 3, target_true_, "Target", makeColor(0.9f, 0.2f, 0.2f)));

    for (auto & marker : markers.markers) {
      marker.header.stamp = stamp;
    }
    marker_pub_->publish(markers);
  }

  void publishStaticTransforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(toTransformStamped(
      "ego_base", "camera_link", T_ego_camera_, this->now()));
    transforms.push_back(toTransformStamped(
      "qr_tag", "target_base", T_target_tag_.inverse(), this->now()));
    static_tf_broadcaster_->sendTransform(transforms);
  }

  void publishTransforms(const rclcpp::Time & stamp)
  {
    T_odom_ego_ = transformFromPose(ego_true_);
    T_odom_camera_ = T_odom_ego_ * T_ego_camera_;
    T_odom_target_ = transformFromPose(target_true_);
    T_odom_tag_ = T_odom_target_ * T_target_tag_;
    T_camera_tag_ = T_odom_camera_.inverse() * T_odom_tag_;

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(toTransformStamped(
      "odom", "ego_base", T_odom_ego_, stamp));
    transforms.push_back(toTransformStamped(
      "camera_link", "qr_tag", T_camera_tag_, stamp));
    tf_broadcaster_->sendTransform(transforms);
  }

  void publishQrPose(const rclcpp::Time & stamp)
  {
    std::normal_distribution<double> noise(0.0, qr_pose_noise_std_);
    const tf2::Vector3 origin = T_camera_tag_.getOrigin();
    const tf2::Quaternion raw_rotation = T_camera_tag_.getRotation();
    const double raw_yaw = std::atan2(
      2.0 * (raw_rotation.w() * raw_rotation.z() +
        raw_rotation.x() * raw_rotation.y()),
      1.0 - 2.0 * (raw_rotation.y() * raw_rotation.y() +
        raw_rotation.z() * raw_rotation.z()));
    const double yaw = raw_yaw + 0.2 * noise(random_engine_);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_link";
    msg.pose.position.x = origin.x() + noise(random_engine_);
    msg.pose.position.y = origin.y() + noise(random_engine_);
    msg.pose.position.z = origin.z() + noise(random_engine_);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    qr_pub_->publish(msg);

    geometry_msgs::msg::PointStamped position_msg;
    position_msg.header = msg.header;
    position_msg.point.x = msg.pose.position.x;
    position_msg.point.y = msg.pose.position.y;
    position_msg.point.z = msg.pose.position.z;
    qr_position_pub_->publish(position_msg);
  }

  Pose2D ego_true_;
  Pose2D target_true_;
  double target_speed_{0.35};
  double target_yaw_rate_{0.0};
  double wheel_radius_{0.254};
  double wheel_track_{0.60};
  double publish_hz_{50.0};
  double measurement_noise_std_{0.008};
  double velocity_noise_std_{0.01};
  double body_length_{0.52};
  double body_width_{0.45};
  double body_height_{0.40};
  double cone_bus_{0.08};
  double qr_pose_noise_std_{0.005};
  double camera_x_{0.35};
  double camera_y_{0.0};
  double camera_z_{0.20};
  double camera_yaw_{0.0};
  double tag_x_{-0.26};
  double tag_y_{0.0};
  double tag_z_{0.05};
  double tag_yaw_{0.0};
  double left_wheel_{0.0};
  double right_wheel_{0.0};

  std::mt19937 random_engine_;

  tf2::Transform T_ego_camera_;
  tf2::Transform T_target_tag_;
  tf2::Transform T_odom_ego_;
  tf2::Transform T_odom_camera_;
  tf2::Transform T_odom_target_;
  tf2::Transform T_odom_tag_;
  tf2::Transform T_camera_tag_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr qr_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ego_yaw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ego_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // 命名空间 docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking::DemoSimNode>());
  rclcpp::shutdown();
  return 0;
}
