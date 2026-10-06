// ArmController 实现
#include "six_axis_control/arm_controller.hpp"

#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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

  // 规划场景接口：内部自管通信节点，构造参数只接受命名空间
  scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

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

bool ArmController::moveNamed(const std::string& name)
{
  RCLCPP_INFO(node_->get_logger(), ">>> 运动到命名姿态：%s", name.c_str());
  move_group_->setNamedTarget(name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(node_->get_logger(), "规划 %s 失败", name.c_str());
    return false;
  }
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
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

bool ArmController::moveCartesianX(double distance, double step)
{
  // 1. 取当前位姿作为起点
  geometry_msgs::msg::Pose start = move_group_->getCurrentPose().pose;

  // 2. 在直线上均匀撒点，每个点朝向都和起点相同
  std::vector<geometry_msgs::msg::Pose> waypoints;
  int n = static_cast<int>(std::abs(distance) / step);
  for (int i = 1; i <= n; ++i) {
    geometry_msgs::msg::Pose p = start;
    p.position.x += distance * static_cast<double>(i) / n;
    waypoints.push_back(p);
  }

  RCLCPP_INFO(node_->get_logger(),
              ">>> 笛卡尔直线：%d 个路径点，沿 x 移动 %.2f m", n, distance);

  // 3. computeCartesianPath：逐点 IK + 直线插值（Jazzy 新签名，无 jump_threshold）
  //    eef_step=step：相邻路径点末端最大间距（米）
  //    trajectory：输出的笛卡尔轨迹（此时只有几何路径，没有时间信息）
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(
      waypoints, step, trajectory);

  RCLCPP_INFO(node_->get_logger(), "笛卡尔路径完成率: %.1f%%", fraction * 100.0);
  if (fraction < 0.9) {
    RCLCPP_ERROR(node_->get_logger(), "路径完成率不足 90%%，放弃执行");
    return false;
  }

  // 4. 执行（execute 有直接接收 RobotTrajectory msg 的重载，
  //    MoveIt 内部自动补时间参数化）
  move_group_->execute(trajectory);
  return true;
}

geometry_msgs::msg::Pose ArmController::currentPose() const
{
  return move_group_->getCurrentPose().pose;
}

bool ArmController::moveToPose(const geometry_msgs::msg::Pose& target)
{
  move_group_->setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(node_->get_logger(), "位姿规划失败（目标不可达或被障碍物封死）");
    return false;
  }
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool ArmController::addBox(const std::string& id,
                          double cx, double cy, double cz,
                          double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "base_link";
  box.id = id;

  // 几何形状：长方体 + 三边尺寸
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {sx, sy, sz};

  // 形状的位姿：盒子中心在 (cx, cy, cz)，朝向与 base_link 一致
  geometry_msgs::msg::Pose pose;
  pose.position.x = cx;
  pose.position.y = cy;
  pose.position.z = cz;
  pose.orientation.w = 1.0;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(pose);
  box.operation = moveit_msgs::msg::CollisionObject::ADD;

  scene_->addCollisionObjects({box});

  // 场景更新通过话题异步传播给 move_group，留时间同步
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(node_->get_logger(),
              ">>> 已添加障碍物 %s：中心(%.2f,%.2f,%.2f) 尺寸(%.2f,%.2f,%.2f)",
              id.c_str(), cx, cy, cz, sx, sy, sz);
  return true;
}

bool ArmController::removeBox(const std::string& id)
{
  scene_->removeCollisionObjects({id});
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(node_->get_logger(), ">>> 已移除障碍物 %s", id.c_str());
  return true;
}

}  // namespace six_axis
