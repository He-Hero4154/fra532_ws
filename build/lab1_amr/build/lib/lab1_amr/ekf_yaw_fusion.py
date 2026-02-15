import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

from tf_transformations import euler_from_quaternion, quaternion_from_euler


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transpose(A):
    return [list(row) for row in zip(*A)]


def matmul(A, B):
    n = len(A)
    m = len(B[0])
    k = len(B)
    out = [[0.0] * m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            s = 0.0
            for kk in range(k):
                s += A[i][kk] * B[kk][j]
            out[i][j] = s
    return out


def matadd(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


class EkfYawFusion(Node):
    """
    EKF state: [x, y, yaw]
    - Prediction from /odom_wheel increments
    - Measurement update from IMU yaw (orientation quaternion)
    - Update yaw only (x,y unaffected directly by IMU)
    """

    def __init__(self):
        super().__init__('ekf_yaw_fusion')

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link_ekf')

        # Process noise (std)
        self.declare_parameter('q_xy_base', 0.01)      # m per step
        self.declare_parameter('q_xy_scale', 0.02)     # m per |ds|
        self.declare_parameter('q_yaw_base', 0.02)     # rad per step
        self.declare_parameter('q_yaw_scale', 0.05)    # rad per |dth|

        # Measurement noise yaw from IMU (std)
        self.declare_parameter('r_yaw', 0.08)          # rad

        # IMU yaw offset handling
        self.declare_parameter('yaw_offset', 0.0)          # rad
        self.declare_parameter('auto_yaw_offset', True)    # auto calibrate once at start

        # Path publish
        self.declare_parameter('publish_path', True)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.q_xy_base = float(self.get_parameter('q_xy_base').value)
        self.q_xy_scale = float(self.get_parameter('q_xy_scale').value)
        self.q_yaw_base = float(self.get_parameter('q_yaw_base').value)
        self.q_yaw_scale = float(self.get_parameter('q_yaw_scale').value)

        self.r_yaw = float(self.get_parameter('r_yaw').value)

        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.auto_yaw_offset = bool(self.get_parameter('auto_yaw_offset').value)
        self.offset_ready = (not self.auto_yaw_offset)

        self.publish_path = bool(self.get_parameter('publish_path').value)

        # EKF state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Covariance P
        self.P = [
            [0.5**2, 0.0,   0.0],
            [0.0,   0.5**2, 0.0],
            [0.0,   0.0,   (20.0 * math.pi / 180.0) ** 2],
        ]

        # Wheel odom prev: (t, xw, yw, yaww)
        self.w_prev = None

        # Latest IMU yaw measurement (after offset)
        self.imu_yaw = None
        self.imu_t = None

        # ROS
        self.sub_odom = self.create_subscription(Odometry, '/odom_wheel', self.on_odom_wheel, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.on_imu, 50)

        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 20)
        self.path_pub = self.create_publisher(Path, '/path_ekf', 5)
        self.tf_br = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        self.get_logger().info(
            f"ekf_yaw_fusion up ✅ frames: {self.odom_frame} -> {self.base_frame}, "
            f"auto_yaw_offset={self.auto_yaw_offset}, r_yaw={self.r_yaw}"
        )

    # ---------------- IMU (measurement) ----------------
    def on_imu(self, msg: Imu):
        q = msg.orientation
        _, _, yaw_raw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_raw = wrap_to_pi(float(yaw_raw))

        # Auto calibrate once: align IMU yaw to wheel yaw at start
        if self.auto_yaw_offset and (not self.offset_ready) and (self.w_prev is not None):
            yaww_prev = self.w_prev[3]
            self.yaw_offset = wrap_to_pi(yaww_prev - yaw_raw)
            self.offset_ready = True
            self.get_logger().info(
                f"Auto yaw_offset set to {self.yaw_offset:.6f} rad ({self.yaw_offset*180/math.pi:.2f} deg)"
            )

        # Apply offset (if not ready yet, yaw_offset may be 0)
        z_yaw = wrap_to_pi(yaw_raw + self.yaw_offset)

        self.imu_yaw = z_yaw
        self.imu_t = Time.from_msg(msg.header.stamp)

        # Measurement update yaw only
        self.measurement_update_yaw(z_yaw)

    # ---------------- Wheel odom (prediction + publish) ----------------
    def on_odom_wheel(self, msg: Odometry):
        t = Time.from_msg(msg.header.stamp)
        xw = float(msg.pose.pose.position.x)
        yw = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        _, _, yaww = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaww = wrap_to_pi(float(yaww))

        if self.w_prev is None:
            self.w_prev = (t, xw, yw, yaww)
            # init EKF near wheel odom
            self.x, self.y, self.yaw = xw, yw, yaww
            self.publish_state(msg.header.stamp)
            return

        t_prev, xw_prev, yw_prev, yaww_prev = self.w_prev
        dt = (t.nanoseconds - t_prev.nanoseconds) * 1e-9
        if dt <= 0.0 or dt < 1e-6:
            self.w_prev = (t, xw, yw, yaww)
            return

        dx = xw - xw_prev
        dy = yw - yw_prev
        dth = wrap_to_pi(yaww - yaww_prev)

        # forward distance (project onto previous heading)
        ds = math.cos(yaww_prev) * dx + math.sin(yaww_prev) * dy

        self.prediction_update(ds, dth)
        self.publish_state(msg.header.stamp)

        self.w_prev = (t, xw, yw, yaww)

    # ---------------- EKF prediction ----------------
    def prediction_update(self, ds: float, dth: float):
        # state prediction
        x = self.x + ds * math.cos(self.yaw)
        y = self.y + ds * math.sin(self.yaw)
        yaw = wrap_to_pi(self.yaw + dth)

        # Jacobian F
        F = [
            [1.0, 0.0, -ds * math.sin(self.yaw)],
            [0.0, 1.0,  ds * math.cos(self.yaw)],
            [0.0, 0.0,  1.0],
        ]

        # Process noise Q
        sig_xy = self.q_xy_base + self.q_xy_scale * abs(ds)
        sig_yaw = self.q_yaw_base + self.q_yaw_scale * abs(dth)
        Q = [
            [sig_xy**2, 0.0,       0.0],
            [0.0,       sig_xy**2, 0.0],
            [0.0,       0.0,       sig_yaw**2],
        ]

        FP = matmul(F, self.P)
        FPFt = matmul(FP, transpose(F))
        self.P = matadd(FPFt, Q)

        self.x, self.y, self.yaw = x, y, yaw

    # ---------------- EKF measurement update (yaw only) ----------------
    def measurement_update_yaw(self, z_yaw: float):
        # innovation
        nu = wrap_to_pi(z_yaw - self.yaw)

        # S = P33 + R
        R = (self.r_yaw ** 2)
        S = self.P[2][2] + R
        if S <= 1e-12:
            return

        # K3 = P33 / S   (update yaw only)
        K3 = self.P[2][2] / S

        # state update (yaw only)
        self.yaw = wrap_to_pi(self.yaw + K3 * nu)

        # covariance update (reduce yaw uncertainty + cross terms with yaw)
        scale = (1.0 - K3)
        # scale column 2 and row 2 to keep symmetry-ish
        for i in range(3):
            self.P[i][2] *= scale
            self.P[2][i] *= scale

        # ensure P33 is correct
        # (after scaling both row/col, P33 becomes scale^2 * old P33; we want scale*old P33)
        # so fix P33 explicitly:
        self.P[2][2] = scale * (self.P[2][2] / (scale if scale != 0 else 1.0))

    # ---------------- publish odom + tf + path ----------------
    def publish_state(self, stamp_msg):
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        tfm = TransformStamped()
        tfm.header.stamp = stamp_msg
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

        if self.publish_path:
            ps = PoseStamped()
            ps.header.stamp = stamp_msg
            ps.header.frame_id = self.odom_frame
            ps.pose = odom.pose.pose

            self.path_msg.header.stamp = stamp_msg
            self.path_msg.poses.append(ps)
            if len(self.path_msg.poses) > 20000:
                self.path_msg.poses = self.path_msg.poses[-20000:]
            self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = EkfYawFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()