from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 安装后 xacro 文件在 share/arm_description/urdf/ 下
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
    )

    # 调用 xacro 命令把 xacro 展开成 urdf 字符串，传给 robot_description 参数
    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([
        # 1. 关节滑条 GUI：拖动滑条发布 /joint_states
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        # 2. 机器人状态发布器：urdf + joint_states -> 计算各连杆 TF 变换
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # 3. RViz 可视化
        Node(
            package="rviz2",
            executable="rviz2",
        ),
    ])
