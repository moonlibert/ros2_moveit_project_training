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

    // 业务流程
    arm.printCurrentPose();     // home 全零位形：末端在正上方
    arm.moveNamed("ready");     // 先走到弯曲姿态，离开伸直奇异点、进入工作空间中部
    arm.printCurrentPose();     // 打印 ready 姿态下的末端位姿
    arm.moveRelativeX(0.10);    // 再沿 x 平移 10cm，此时 IK 有解
  }
  // 离开作用域 → arm 析构 → 自动 cancel/join，资源干净释放

  rclcpp::shutdown();
  return 0;
}
