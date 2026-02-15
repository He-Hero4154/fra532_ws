#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class SlamPoseFromTF(Node):
    def __init__(self):
        super().__init__("slam_pose_from_tf")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link_icp").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom_slam").value
        self.path_topic = self.declare_parameter("path_topic", "/path_slam").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 5.0).value)
        self.max_path_len = int(self.declare_parameter("max_path_len", 20000).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_path = self.create_publisher(Path, self.path_topic, 5)

        self.path = Path()
        self.path.header.frame_id = str(self.map_frame)

        period = 1.0 / max(0.1, self.rate_hz)
        self.create_timer(period, self.on_timer)

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.map_frame),
                str(self.base_frame),
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return

        odom = Odometry()
        odom.header.stamp = tf.header.stamp
        odom.header.frame_id = str(self.map_frame)
        odom.child_frame_id = str(self.base_frame)
        odom.pose.pose.position.x = tf.transform.translation.x
        odom.pose.pose.position.y = tf.transform.translation.y
        odom.pose.pose.position.z = tf.transform.translation.z
        odom.pose.pose.orientation = tf.transform.rotation
        self.pub_odom.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.path.header.stamp = odom.header.stamp
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_path_len:
            self.path.poses = self.path.poses[-self.max_path_len:]
        self.pub_path.publish(self.path)


def main():
    rclpy.init()
    node = SlamPoseFromTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()