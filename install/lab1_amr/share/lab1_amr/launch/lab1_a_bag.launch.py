from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    use_rviz = LaunchConfiguration("use_rviz")

    # relative paths inside share/<pkg> (เราจะทำให้ setup.py install ให้ในขั้นถัดไป)
    wheel_yaml_rel = LaunchConfiguration("wheel_yaml")
    ekf_yaml_rel = LaunchConfiguration("ekf_yaml")
    icp_yaml_rel = LaunchConfiguration("icp_yaml")
    rviz_cfg_rel = LaunchConfiguration("rviz_cfg")

    pkg_share = FindPackageShare("lab1_amr")

    wheel_yaml = PathJoinSubstitution([pkg_share, wheel_yaml_rel])
    ekf_yaml = PathJoinSubstitution([pkg_share, ekf_yaml_rel])
    icp_yaml = PathJoinSubstitution([pkg_share, icp_yaml_rel])
    rviz_cfg = PathJoinSubstitution([pkg_share, rviz_cfg_rel])

    # 1) play bag (+ /clock) เริ่มทันที
    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", rate],
        output="screen",
    )

    # 2) nodes (หน่วงนิดนึงให้ /clock มาแน่ ๆ)
    wheel_node = Node(
        package="lab1_amr",
        executable="wheel_odom",
        name="wheel_odom",
        output="screen",
        parameters=[wheel_yaml, {"use_sim_time": True}],
    )

    ekf_node = Node(
        package="lab1_amr",
        executable="ekf_yaw_fusion",
        name="ekf_yaw_fusion",
        output="screen",
        parameters=[ekf_yaml, {"use_sim_time": True}],
    )

    icp_node = Node(
        package="lab1_amr",
        executable="icp_odom",
        name="icp_odom",
        output="screen",
        parameters=[icp_yaml, {"use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="/home/hero/fra532_ws/data/FRA532_LAB1_DATASET/fibo_floor3_seq00",
                description="Path to rosbag2 folder",
            ),
            DeclareLaunchArgument("rate", default_value="1.0", description="rosbag play rate"),
            DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2"),

            DeclareLaunchArgument("wheel_yaml", default_value="config/wheel_params.yaml"),
            DeclareLaunchArgument("ekf_yaml", default_value="config/ekf_params.yaml"),
            DeclareLaunchArgument("icp_yaml", default_value="config/icp_params.yaml"),
            DeclareLaunchArgument("rviz_cfg", default_value="rviz/lab1.rviz"),

            bag_play,

            TimerAction(period=0.5, actions=[wheel_node, ekf_node, icp_node]),
            TimerAction(period=1.0, actions=[rviz_node]),
        ]
    )