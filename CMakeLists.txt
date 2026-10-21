cmake_minimum_required(VERSION 3.8)
project(Docking_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(docking_core
  src/target_state_estimator.cpp
  src/target_predictor.cpp
  src/reference_line_generator.cpp
  src/local_planner.cpp
  src/speed_planner.cpp
  src/docking_state_machine.cpp
  src/differential_controller.cpp
  src/relative_state_estimator.cpp
)

target_include_directories(docking_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(docking_planner_node
  src/docking_planner_main.cpp
  src/docking_planner_node.cpp
)
target_link_libraries(docking_planner_node docking_core)
ament_target_dependencies(docking_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
)

add_executable(demo_sim_node
  src/demo_sim_node.cpp
)
target_link_libraries(demo_sim_node docking_core)
ament_target_dependencies(demo_sim_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(qr_target_adapter_node
  src/qr_target_adapter_node.cpp
)
target_link_libraries(qr_target_adapter_node docking_core)
ament_target_dependencies(qr_target_adapter_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  docking_core
  docking_planner_node
  demo_sim_node
  qr_target_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
