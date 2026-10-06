// 阶段4：笛卡尔直线运动
// 先去 ready 姿态（离开奇异点），再让末端沿 x 严格走 15cm 直线
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "six_axis_control/arm_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  {
    six_axis::ArmController arm;

    arm.moveNamed("ready");       // 进入工作空间中部
    arm.moveCartesianX(0.15);     // 末端沿 x 走直线 15cm（15 个路径点）
  }

  rclcpp::shutdown();
  return 0;
}
