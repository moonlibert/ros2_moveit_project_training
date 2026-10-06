// 阶段3（OOP 版）：main 只负责组装和调用业务流程
// 所有 ROS / MoveIt 细节都封装在 ArmController 类里
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "six_axis_control/arm_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  {
    // 构造即就绪：node、spin 线程、MoveGroupInterface 全部启动
    six_axis::ArmController arm;

    // 业务逻辑只剩两行，意图一目了然
    arm.printCurrentPose();
    arm.moveRelativeX(0.10);
  }
  // 离开作用域 → arm 析构 → 自动 cancel/join，资源干净释放

  rclcpp::shutdown();
  return 0;
}
