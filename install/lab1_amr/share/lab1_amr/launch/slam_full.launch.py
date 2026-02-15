import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_lab = get_package_share_directory("lab1_amr")
    pkg_slam = get_package_share_directory("slam_toolbox")

    # your configs
    wheel_params = os.path.join(pkg_lab, "config", "wheel_params.yaml")
    ekf_params = os.path.join(pkg_lab, "config", "ekf_params.yaml")
    icp_params = os.path.join(pkg_lab, "config", "icp_params.yaml")

    # slam toolbox default config fallback
    slam_default = os.path.join(pkg_slam, "config", "mapper_params_online_async.yaml")
    slam_params = slam_default  # ใช้ของ slam_toolbox ไปก่อนก็ได้

    default_rviz = os.path.join(pkg_lab, "rviz", "lab1.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    bag_path = LaunchConfiguration("bag_path")
    rviz_config = LaunchConfiguration("rviz_config")

    # SLAM frames/topics
    base_frame = LaunchConfiguration("base_frame")  # ใช้ ICP เป็นฐาน
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    scan_topic = LaunchConfiguration("scan_topic")

    # static TF base_frame -> base_scan (จำเป็นมาก เพราะ dataset ไม่มี /tf_static ให้)
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    static_tf_x = LaunchConfiguration("static_tf_x")
    static_tf_y = LaunchConfiguration("static_tf_y")
    static_tf_z = LaunchConfiguration("static_tf_z")
    static_tf_yaw = LaunchConfiguration("static_tf_yaw")

    # 0) play bag (หน่วงนิดให้ node พร้อมก่อน)
    play_bag = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bag_path, "' != ''"])),
    )

    play_bag_delayed = TimerAction(period=2.0, actions=[play_bag])

    # 1) wheel odom (สร้าง odom->base_link_wheel)
    wheel = Node(
        package="lab1_amr",
        executable="wheel_odom",
        name="wheel_odom",
        output="screen",
        parameters=[wheel_params, {"use_sim_time": use_sim_time}],
    )

    # 2) EKF yaw fusion (สร้าง odom->base_link_ekf, /odom_ekf)
    ekf = Node(
        package="lab1_amr",
        executable="ekf_yaw_fusion",
        name="ekf_yaw_fusion",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
    )

    # 3) ICP odom (สร้าง odom->base_link_icp, /odom_icp และ /path_icp)
    icp = Node(
        package="lab1_amr",
        executable="icp_odom",
        name="icp_odom",
        output="screen",
        parameters=[icp_params, {"use_sim_time": use_sim_time}],
    )

    # 4) static TF: base_link_icp -> base_scan (ถ้าไม่ใส่ slam จะบ้าทันที)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_scan",
        output="screen",
        arguments=[
            static_tf_x,
            static_tf_y,
            static_tf_z,
            "0.0",
            "0.0",
            static_tf_yaw,
            base_frame,
            "base_scan",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_static_tf),
    )

    # 5) slam_toolbox (หน่วงอีกนิดให้ TF/odom มาก่อน)
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "base_frame": base_frame,
                "odom_frame": odom_frame,
                "map_frame": map_frame,
                "scan_topic": scan_topic,
            },
        ],
    )
    slam_delayed = TimerAction(period=3.0, actions=[slam])

    # 6) RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),

            DeclareLaunchArgument("base_frame", default_value="base_link_icp"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),

            # เปิดไว้เลย ไม่งั้น base_scan ไม่มี TF
            DeclareLaunchArgument("publish_static_tf", default_value="true"),
            DeclareLaunchArgument("static_tf_x", default_value="0.0"),
            DeclareLaunchArgument("static_tf_y", default_value="0.0"),
            DeclareLaunchArgument("static_tf_z", default_value="0.0"),
            DeclareLaunchArgument("static_tf_yaw", default_value="0.0"),

            wheel,
            ekf,
            icp,
            static_tf,
            play_bag_delayed,
            slam_delayed,
            rviz,
        ]
    )