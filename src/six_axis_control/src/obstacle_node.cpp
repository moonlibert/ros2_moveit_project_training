// 阶段5：避障规划
// 走到 ready → 在末端正前方立一堵薄墙 → 给墙后面的目标位姿
// OMPL 会自动搜索一条绕过墙的路径
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "six_axis_control/arm_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  {
    six_axis::ArmController arm;

    // 1. 先到工作空间中部的 ready 姿态
    arm.moveNamed("ready");
    auto p = arm.currentPose();

    // 2. 在末端前方 0.12m 处立墙：厚 0.02（x）、宽 0.6（y）、高 0.25（z），
    //    墙中心与当前末端同高，刚好挡住水平直线路径
    const double wall_x = p.position.x + 0.12;
    arm.addBox("wall",
               wall_x, p.position.y, p.position.z,
               0.02, 0.60, 0.25);

    // 3. 目标在墙的另一侧（再往前 0.30m），朝向保持不变
    geometry_msgs::msg::Pose target = p;
    target.position.x += 0.30;

    // 4. 自由空间规划：MoveIt 已知墙的存在，轨迹会从墙上方（或侧面）绕过
    arm.moveToPose(target);

    // 5. 演示完移除障碍物
    arm.removeBox("wall");
  }

  rclcpp::shutdown();
  return 0;
}
