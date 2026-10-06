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
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# 暴露 include/ 目录，让源码能 #include "six_axis_control/xxx.hpp"
include_directories(include)

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

# ---- 阶段3：OOP 封装库 + 节点 ----
# arm_controller 编成库，阶段4/5 的节点可直接复用
add_library(arm_controller_lib src/arm_controller.cpp)
target_include_directories(arm_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_controller_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  moveit_msgs
  shape_msgs
)

add_executable(pose_target_oop src/pose_target_oop_node.cpp)
target_link_libraries(pose_target_oop arm_controller_lib)
ament_target_dependencies(pose_target_oop rclcpp)

# ---- 阶段4：笛卡尔直线节点 ----
add_executable(cartesian src/cartesian_node.cpp)
target_link_libraries(cartesian arm_controller_lib)
ament_target_dependencies(cartesian rclcpp)

# ---- 阶段5：避障节点 ----
add_executable(obstacle src/obstacle_node.cpp)
target_link_libraries(obstacle arm_controller_lib)
ament_target_dependencies(obstacle rclcpp geometry_msgs)

install(TARGETS named_target joint_target pose_target_oop cartesian obstacle arm_controller_lib
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件，供其他包复用
install(DIRECTORY include/
  DESTINATION include
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
