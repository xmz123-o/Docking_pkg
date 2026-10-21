#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "docking_pkg/docking_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking::DockingPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
