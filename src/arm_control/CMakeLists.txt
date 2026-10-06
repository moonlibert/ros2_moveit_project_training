cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(motion_planner src/motion_planner_node.cpp)
target_compile_features(motion_planner PRIVATE cxx_std_17)
ament_target_dependencies(motion_planner
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
