import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

from tf_transformations import quaternion_from_euler


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # ===== Params (แก้ทีหลังได้ตอน run) =====
        self.declare_parameter('wheel_radius', 0.033)       # m (เริ่มด้วยค่า default)
        self.declare_parameter('wheel_separation', 0.160)   # m (จากรูปของนาย)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link_wheel')
        self.declare_parameter('left_joint', 'wheel_left_joint')
        self.declare_parameter('right_joint', 'wheel_right_joint')
        self.declare_parameter('publish_path', True)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.left_joint = str(self.get_parameter('left_joint').value)
        self.right_joint = str(self.get_parameter('right_joint').value)
        self.publish_path = bool(self.get_parameter('publish_path').value)

        # ===== State =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.phi_L_prev = None
        self.phi_R_prev = None
        self.t_prev = None

        # ===== ROS I/O =====
        self.sub = self.create_subscription(JointState, '/joint_states', self.on_joint, 50)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 20)
        self.path_pub = self.create_publisher(Path, '/path_wheel', 5)
        self.tf_br = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        self.get_logger().info(
            f"wheel_odom up ✅ r={self.r:.4f} m, L={self.L:.4f} m, "
            f"frames: {self.odom_frame} -> {self.base_frame}"
        )

    def on_joint(self, msg: JointState):
        # --- Map joint name -> position (rad) ---
        if not msg.name or not msg.position:
            return

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        if self.left_joint not in name_to_pos or self.right_joint not in name_to_pos:
            self.get_logger().warn_once(
                f"Joint names not found. Need '{self.left_joint}' and '{self.right_joint}'. "
                f"Got: {msg.name}"
            )
            return

        phi_L = float(name_to_pos[self.left_joint])
        phi_R = float(name_to_pos[self.right_joint])

        # --- Time ---
        t = Time.from_msg(msg.header.stamp)
        if self.t_prev is None:
            # first message: initialize
            self.phi_L_prev = phi_L
            self.phi_R_prev = phi_R
            self.t_prev = t
            return

        dt = (t.nanoseconds - self.t_prev.nanoseconds) * 1e-9
        if dt <= 0.0 or dt < 1e-6:
            # skip weird timestamps
            self.phi_L_prev = phi_L
            self.phi_R_prev = phi_R
            self.t_prev = t
            return

        # --- Wheel increments ---
        dphi_L = phi_L - self.phi_L_prev
        dphi_R = phi_R - self.phi_R_prev

        ds_L = self.r * dphi_L
        ds_R = self.r * dphi_R

        ds = 0.5 * (ds_R + ds_L)
        dth = (ds_R - ds_L) / self.L

        # --- Integrate pose (exact for constant curvature) ---
        if abs(dth) < 1e-9:
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
        else:
            # ICC integration
            self.x += (ds / dth) * (math.sin(self.yaw + dth) - math.sin(self.yaw))
            self.y += (ds / dth) * (-math.cos(self.yaw + dth) + math.cos(self.yaw))

        self.yaw = wrap_to_pi(self.yaw + dth)

        v = ds / dt
        w = dth / dt

        # --- Publish odom ---
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # --- TF: odom -> base_link_wheel ---
        tfm = TransformStamped()
        tfm.header.stamp = msg.header.stamp
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = self.base_frame
        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = q[0]
        tfm.transform.rotation.y = q[1]
        tfm.transform.rotation.z = q[2]
        tfm.transform.rotation.w = q[3]
        self.tf_br.sendTransform(tfm)

        # --- Publish path (optional, but RViz-friendly) ---
        if self.publish_path:
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = self.odom_frame
            ps.pose = odom.pose.pose

            self.path_msg.header.stamp = msg.header.stamp
            self.path_msg.poses.append(ps)

            # กัน path โตจนบวม (เอาไว้ดูเฉย ๆ)
            if len(self.path_msg.poses) > 20000:
                self.path_msg.poses = self.path_msg.poses[-20000:]

            self.path_pub.publish(self.path_msg)

        # --- Update prev ---
        self.phi_L_prev = phi_L
        self.phi_R_prev = phi_R
        self.t_prev = t


def main():
    rclpy.init()
    node = WheelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    