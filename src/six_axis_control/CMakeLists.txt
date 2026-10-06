cmake_minimum_required(VERSION 3.8)
project(six_axis_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

# ---- 阶段1：命名姿态控制节点 ----
add_executable(named_target src/named_target_node.cpp)
ament_target_dependencies(named_target
  rclcpp
  moveit_ros_planning_interface
  moveit_core
)

# ---- 阶段2：关节空间目标节点 ----
add_executable(joint_target src/joint_target_node.cpp)
ament_target_dependencies(joint_target
  rclcpp
  moveit_ros_planning_interface
  moveit_core
)

install(TARGETS named_target joint_target
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in this repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
