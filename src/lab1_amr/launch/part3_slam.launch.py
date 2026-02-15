from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lab1_amr')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    static_tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_icp_to_base_scan',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link_icp',
            '--child-frame-id', 'base_scan',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen'
    )

    return LaunchDescription([
        static_tf_scan,
        slam,
    ])