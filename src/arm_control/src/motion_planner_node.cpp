// C++ MoveIt 2 运动规划演示节点
// 运行前提：demo.launch.py 已启动（提供 move_group + 虚拟硬件控制器）
//   ros2 run arm_control motion_planner

#include <map>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_planner");
  const auto logger = node->get_logger();

  // MoveGroupInterface 内部通过 action/topic 与 move_group 通信，
  // 必须有 executor 在独立线程中持续 spin 处理回调。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // 连接 SRDF 中定义的 "arm" 规划组（4 个关节 joint1~joint4）
  MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.1);      // 限速 10%，练习时动作平缓
  arm.setMaxAccelerationScalingFactor(0.1);

  RCLCPP_INFO(logger, "规划组: %s | 末端参考坐标系: %s",
              arm.getName().c_str(),
              arm.getPlanningFrame().c_str());

  // 统一的"规划并执行"辅助函数：先 plan 再 execute
  auto plan_and_execute = [&]() -> bool {
    MoveGroupInterface::Plan plan;
    if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "规划失败");
      return false;
    }
    RCLCPP_INFO(logger, "规划成功，轨迹含 %zu 个点",
                plan.trajectory.joint_trajectory.points.size());
    return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  };

  // ========== 动作 1：运动到 SRDF 里预定义的 home 姿态 ==========
  RCLCPP_INFO(logger, ">>> 动作1：回到 home 姿态");
  arm.setNamedTarget("home");
  if (!plan_and_execute()) {
    RCLCPP_ERROR(logger, "回到 home 失败，退出");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ========== 动作 2：关节空间目标（直接指定 4 个关节角，弧度） ==========
  RCLCPP_INFO(logger, ">>> 动作2：运动到指定关节角");
  const std::map<std::string, double> joint_goal = {
      {"joint1", 0.5},
      {"joint2", -0.2},
      {"joint3", 0.7},
      {"joint4", -0.3},
  };
  arm.setJointValueTarget(joint_goal);
  plan_and_execute();

  // ========== 动作 3：打印当前末端（tool0）位姿，再回 home ==========
  RCLCPP_INFO(logger, ">>> 动作3：读取当前末端位姿");
  geometry_msgs::msg::PoseStamped current_pose = arm.getCurrentPose("tool0");
  RCLCPP_INFO(logger, "末端位置 -> x: %.3f  y: %.3f  z: %.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  RCLCPP_INFO(logger, ">>> 返回 home，演示结束");
  arm.setNamedTarget("home");
  plan_and_execute();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
