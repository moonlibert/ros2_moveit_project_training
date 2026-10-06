// ArmController：机械臂控制封装类
// 把 node / executor / spin 线程 / MoveGroupInterface 收进一个对象，
// 构造即就绪，析构自动回收资源（RAII）。
#pragma once

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace six_axis {

class ArmController {
public:
  // 构造函数：完成所有初始化（要求 main 里已 rclcpp::init）
  ArmController();

  // 析构函数：停 spin、收线程，自动调用
  ~ArmController();

  // 禁止拷贝（持有线程和 ROS 资源，拷贝无意义且危险）
  ArmController(const ArmController&) = delete;
  ArmController& operator=(const ArmController&) = delete;

  // 打印 tool0 当前位姿，返回是否成功拿到数据
  bool printCurrentPose();

  // 在当前位置基础上沿 base_link 的 x 轴平移 dx（米），朝向不变
  bool moveRelativeX(double dx);

  // 对外暴露 node（以后加订阅/发布/定时器时用）
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  // 成员声明顺序 = 构造顺序，node 必须最先（move_group 依赖它）
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  static constexpr const char* PLANNING_GROUP = "arm_6axises";
};

}  // namespace six_axis
