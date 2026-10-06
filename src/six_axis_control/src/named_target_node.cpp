// 阶段1：命名姿态控制
// 在 SRDF 里预定义的两个姿态 home / ready 之间来回切换
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("named_target_node");

  // MoveGroupInterface 是 C++ 操作机械臂的主入口。
  // 构造时传入节点 + 规划组名字（必须和 SRDF 里的 <group name="..."> 完全一致）。
  static const std::string PLANNING_GROUP = "arm_6axises";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 规划出的轨迹先放在 plan 里，确认成功后再 execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // ---- 动作1：回到 home（全零位） ----
  RCLCPP_INFO(node->get_logger(), ">>> 目标：home");
  move_group.setNamedTarget("home");  // 名字来自 SRDF 的 <group_state name="home">
  bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "规划 home: %s", ok ? "成功" : "失败");
  if (ok) {
    move_group.execute(plan);
  }

  // 给点时间让动作走完
  rclcpp::sleep_for(std::chrono::seconds(2));

  // ---- 动作2：运动到 ready（SRDF 里预设的另一组关节角） ----
  RCLCPP_INFO(node->get_logger(), ">>> 目标：ready");
  move_group.setNamedTarget("ready");
  ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "规划 ready: %s", ok ? "成功" : "失败");
  if (ok) {
    move_group.execute(plan);
  }

  rclcpp::shutdown();
  return 0;
}
