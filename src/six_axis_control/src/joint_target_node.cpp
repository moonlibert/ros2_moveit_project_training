// 阶段2：关节空间目标
// 1) 读取并打印当前关节角
// 2) 直接给一组目标角度
// 3) 在当前姿态基础上做"相对运动"
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("joint_target_node");

  // Jazzy 起 MoveGroupInterface 不再内部 spin 用户节点，
  // 必须自己开 executor + 线程持续 spin，否则 /joint_states 等订阅回调不执行。
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() { executor->spin(); });

  static const std::string PLANNING_GROUP = "arm_6axises";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 构造后状态监视器刚启动，/joint_states 可能还没收到。
  // 等待当前关节角非空，最多等 5 秒，避免读到空 vector 越界崩溃。
  std::vector<double> current = move_group.getCurrentJointValues();
  for (int i = 0; i < 50 && current.empty(); ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    current = move_group.getCurrentJointValues();
  }
  if (current.empty()) {
    RCLCPP_ERROR(node->get_logger(), "等不到 /joint_states，确认 demo.launch.py 是否在运行");
    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // ---- 1. 读取当前关节角 ----
  // Jazzy 中 getCurrentJointValues() 返回 vector<double>，顺序由 getJoints() 决定，
  // 数据源头是 /joint_states
  std::vector<std::string> joints = move_group.getJoints();
  RCLCPP_INFO(node->get_logger(), ">>> 当前关节角（弧度）：");
  for (size_t i = 0; i < joints.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  %s = %.3f", joints[i].c_str(), current[i]);
  }

  // 封装一个"规划并执行"的小函数（lambda），后面复用
  auto go = [&](const std::string& tag) {
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "规划 %s: %s", tag.c_str(), ok ? "成功" : "失败");
    if (ok) move_group.execute(plan);
  };

  // ---- 2. 直接指定每个关节的目标角度 ----
  // 用 关节名->角度 的 map 给目标，想动哪个动哪个，其余关节保持
  RCLCPP_INFO(node->get_logger(), ">>> 绝对目标：转腰 + 抬肩");
  move_group.setJointValueTarget(std::map<std::string, double>{
    {"joint1",  0.5},    // 腰部转 0.5 弧度（约 29 度）
    {"joint2", -0.5},    // 肩部抬起
    {"joint3",  0.8},    // 肘部弯曲
    {"joint4",  0.0},
    {"joint5",  0.0},
    {"joint6",  0.0},
  });
  go("绝对目标");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // ---- 3. 相对运动：在当前角度基础上，每个关节 +0.15 弧度 ----
  current = move_group.getCurrentJointValues();
  std::vector<double> target = current;  // 以当前值为起点拷贝一份
  for (double& angle : target) {
    angle += 0.15;                       // 只改目标，current 保持原样
  }
  RCLCPP_INFO(node->get_logger(), ">>> 相对目标：所有关节 +0.15 弧度");
  move_group.setJointValueTarget(target);
  go("相对目标");

  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
