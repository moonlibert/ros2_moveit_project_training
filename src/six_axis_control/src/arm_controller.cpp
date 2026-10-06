// ArmController 实现
#include "six_axis_control/arm_controller.hpp"

#include <chrono>

#include <geometry_msgs/msg/pose.hpp>

namespace six_axis {

ArmController::ArmController()
{
  // 1. 通信底座
  node_ = std::make_shared<rclcpp::Node>("pose_target_oop_node");

  // 2. executor + spin 线程（必须在 move_group 构造前就绪，
  //    这样 move_group 一订阅 /joint_states，回调立刻有人处理）
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  // 3. MoveIt 客户端（延迟到 node/spin 就绪后再构造）
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, PLANNING_GROUP);
  move_group_->setPoseReferenceFrame("base_link");
  move_group_->setEndEffectorLink("tool0");
  move_group_->setPlanningTime(5.0);

  // 4. 等待第一帧关节状态（避免读到空缓存）
  auto values = move_group_->getCurrentJointValues();
  for (int i = 0; i < 50 && values.empty(); ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    values = move_group_->getCurrentJointValues();
  }
  if (values.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "等不到 /joint_states，demo 是否在运行？");
  }
}

ArmController::~ArmController()
{
  // 逆序清理：先停 executor，再 join 线程
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  // move_group_ 是 unique_ptr，自动析构
  // node_ 是 shared_ptr，最后一个引用释放时自动析构
}

bool ArmController::printCurrentPose()
{
  auto pose = move_group_->getCurrentPose();
  if (pose.header.frame_id.empty()) {
    RCLCPP_WARN(node_->get_logger(), "当前位姿不可用");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), ">>> tool0 位姿（相对 %s）：",
              pose.header.frame_id.c_str());
  RCLCPP_INFO(node_->get_logger(), "  位置 x=%.3f y=%.3f z=%.3f",
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "  朝向 qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
              pose.pose.orientation.x,
              pose.pose.orientation.y,
              pose.pose.orientation.z,
              pose.pose.orientation.w);
  return true;
}

bool ArmController::moveRelativeX(double dx)
{
  // 以当前位姿为起点，只改 x
  geometry_msgs::msg::Pose target = move_group_->getCurrentPose().pose;
  target.position.x += dx;

  RCLCPP_INFO(node_->get_logger(), ">>> 末端沿 x 平移 %.2f m", dx);
  move_group_->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(node_->get_logger(), "规划失败（目标可能不可达）");
    return false;
  }
  auto err = move_group_->execute(plan);
  return (err == moveit::core::MoveItErrorCode::SUCCESS);
}

}  // namespace six_axis
