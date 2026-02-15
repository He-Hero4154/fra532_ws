#!/usr/bin/env python3
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


# ----------------------------
# helpers
# ----------------------------
def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)


def se2_to_rt(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    R = np.array([[c, -s], [s, c]], dtype=np.float64)
    t = np.array([x, y], dtype=np.float64)
    return R, t


def rt_to_se2(R, t):
    yaw = math.atan2(R[1, 0], R[0, 0])
    return float(t[0]), float(t[1]), float(yaw)


def se2_compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    Ra, ta = se2_to_rt(ax, ay, ath)
    Rb, tb = se2_to_rt(bx, by, bth)
    R = Ra @ Rb
    t = Ra @ tb + ta
    x, y, th = rt_to_se2(R, t)
    return x, y, wrap_to_pi(th)


def se2_inverse(a):
    x, y, th = a
    R, t = se2_to_rt(x, y, th)
    Rinv = R.T
    tinv = -Rinv @ t
    xi, yi, thi = rt_to_se2(Rinv, tinv)
    return xi, yi, wrap_to_pi(thi)


def se2_between(a, b):
    return se2_compose(se2_inverse(a), b)


# ----------------------------
# ICP core (2D point-to-point)
# ----------------------------
def best_fit_transform_2d(src: np.ndarray, dst: np.ndarray):
    if src.shape[0] < 3:
        return np.eye(2), np.zeros(2)

    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src0 = src - mu_s
    dst0 = dst - mu_d

    H = src0.T @ dst0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = mu_d - R @ mu_s
    return R, t


def icp_2d(
    src: np.ndarray,
    dst: np.ndarray,
    R0: np.ndarray,
    t0: np.ndarray,
    iters: int = 25,
    max_corr: float = 0.25,
    tol_trans: float = 1e-4,
    tol_rot: float = 1e-4,
):
    if src.shape[0] < 10 or dst.shape[0] < 10:
        return R0, t0, float("inf"), 0

    R = R0.copy()
    t = t0.copy()
    max_corr2 = max_corr * max_corr

    rmse = float("inf")
    pairs = 0

    for _ in range(iters):
        src_t = (src @ R.T) + t

        diff = src_t[:, None, :] - dst[None, :, :]
        d2 = np.sum(diff * diff, axis=2)
        nn_idx = np.argmin(d2, axis=1)
        nn_d2 = d2[np.arange(d2.shape[0]), nn_idx]

        mask = nn_d2 < max_corr2
        inliers = int(np.count_nonzero(mask))
        if inliers < 10:
            return R, t, float("inf"), inliers

        src_m = src_t[mask]
        dst_m = dst[nn_idx[mask]]

        dR, dt = best_fit_transform_2d(src_m, dst_m)

        R_new = dR @ R
        t_new = dR @ t + dt

        dth = math.atan2(dR[1, 0], dR[0, 0])
        dtrans = float(np.linalg.norm(dt))
        rmse = float(math.sqrt(np.mean(nn_d2[mask])))
        pairs = inliers

        R, t = R_new, t_new

        if dtrans < tol_trans and abs(dth) < tol_rot:
            break

    return R, t, rmse, pairs


# ----------------------------
# Node
# ----------------------------
class IcpOdomNode(Node):
    def __init__(self):
        super().__init__("icp_odom")

        # topics / frames
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.ekf_topic = self.declare_parameter("ekf_topic", "/odom_ekf").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.child_frame = self.declare_parameter("child_frame", "base_link_icp").value
        self.out_odom_topic = self.declare_parameter("out_odom_topic", "/odom_icp").value
        self.out_path_topic = self.declare_parameter("out_path_topic", "/path_icp").value

        # time sync
        self.sync_tolerance = float(self.declare_parameter("sync_tolerance", 0.8).value)

        # scan preprocessing
        self.min_range = float(self.declare_parameter("min_range", 0.20).value)
        self.max_range = float(self.declare_parameter("max_range", 3.20).value)
        self.fov_deg = float(self.declare_parameter("fov_deg", 270.0).value)  # keep front 270 deg by default
        self.downsample_step = int(self.declare_parameter("downsample_step", 1).value)

        # ICP params (coarse-to-fine)
        self.iters_coarse = int(self.declare_parameter("iters_coarse", 12).value)
        self.max_corr_large = float(self.declare_parameter("max_corr_large", 0.40).value)
        self.icp_iters = int(self.declare_parameter("icp_iters", 25).value)
        self.max_corr_dist = float(self.declare_parameter("max_corr_dist", 0.25).value)

        # yaw search for reacquire
        self.yaw_search_step = float(self.declare_parameter("yaw_search_step", 0.08).value)  # rad ~4.6deg
        self.yaw_search_levels = int(self.declare_parameter("yaw_search_levels", 2).value)  # try 0, ±1, ±2 steps

        # accept criteria
        self.rmse_max = float(self.declare_parameter("rmse_max", 0.12).value)
        self.min_inliers = int(self.declare_parameter("min_inliers", 25).value)
        self.min_inlier_ratio = float(self.declare_parameter("min_inlier_ratio", 0.10).value)

        # consistency gate vs EKF (กัน ICP หลอก)
        self.gate_trans = float(self.declare_parameter("gate_trans", 0.25).value)  # m per scan
        self.gate_rot = float(self.declare_parameter("gate_rot", 0.35).value)      # rad per scan

        # soft correction
        self.alpha = float(self.declare_parameter("alpha", 0.25).value)
        self.res_clip_trans = float(self.declare_parameter("res_clip_trans", 0.08).value)
        self.res_clip_rot = float(self.declare_parameter("res_clip_rot", 0.20).value)

        # reset logic
        self.reset_on_time_jump = bool(self.declare_parameter("reset_on_time_jump", True).value)
        self.max_dt_gap = float(self.declare_parameter("max_dt_gap", 1.0).value)
        self.reject_reset_N = int(self.declare_parameter("reject_reset_N", 20).value)
        self.clear_path_on_reset = bool(self.declare_parameter("clear_path_on_reset", True).value)

        # path
        self.path_max_len = int(self.declare_parameter("path_max_len", 4000).value)

        # pub/sub
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data  # BEST_EFFORT ok with bag scan
        )
        self.sub_ekf = self.create_subscription(Odometry, self.ekf_topic, self.on_ekf, 10)

        self.pub_odom = self.create_publisher(Odometry, self.out_odom_topic, 10)
        self.pub_path = self.create_publisher(Path, self.out_path_topic, 10)
        self.tf_br = TransformBroadcaster(self)

        # buffers/state
        self.ekf_buf = deque(maxlen=6000)  # (Time, (x,y,yaw))
        self.prev_scan_pts = None
        self.prev_scan_t = None
        self.prev_ekf_pose = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.initialized = False

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        self._last = {}
        self.reject_streak = 0

        self.get_logger().info(
            f"icp_odom up ✅ scan={self.scan_topic}, ekf={self.ekf_topic}, frames: {self.odom_frame}->{self.child_frame}, "
            f"coarse(max_corr={self.max_corr_large}, iters={self.iters_coarse}) fine(max_corr={self.max_corr_dist}, iters={self.icp_iters})"
        )

    def _throttle(self, key: str, period: float) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last.get(key, -1e9)
        if (now - last) >= period:
            self._last[key] = now
            return True
        return False

    def reset_state(self, reason: str, ekf_pose=None):
        if self._throttle("reset", 1.0):
            self.get_logger().warn(f"Reset ICP state: {reason}")
        self.prev_scan_pts = None
        self.prev_scan_t = None
        self.prev_ekf_pose = None
        self.initialized = False
        self.reject_streak = 0
        if ekf_pose is not None:
            self.x, self.y, self.yaw = ekf_pose
        if self.clear_path_on_reset:
            self.path_msg.poses = []

    def on_ekf(self, msg: Odometry):
        t = Time.from_msg(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.ekf_buf.append((t, (float(p.x), float(p.y), float(yaw))))

    def _find_ekf_pose_near(self, t_scan: Time):
        if not self.ekf_buf:
            return None, None
        best_pose = None
        best_dt = None
        for t, pose in reversed(self.ekf_buf):
            dt = abs((t.nanoseconds - t_scan.nanoseconds)) * 1e-9
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_pose = pose
            if best_dt is not None and dt > best_dt + 0.5:
                break
        if best_dt is None or best_dt > self.sync_tolerance:
            return None, best_dt
        return best_pose, best_dt

    def scan_to_points(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=np.float64)
        n = ranges.shape[0]
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # wrap angles to [-pi, pi] for FOV cut
        ang = np.array([wrap_to_pi(a) for a in angles], dtype=np.float64)
        fov = math.radians(float(self.fov_deg))
        half = 0.5 * fov

        mask = (
            np.isfinite(ranges)
            & (ranges > self.min_range)
            & (ranges < min(self.max_range, scan.range_max))
            & (ranges > 0.0)
            & (np.abs(ang) <= half)
        )

        ranges = ranges[mask]
        ang = ang[mask]

        if ranges.size < 20:
            return np.zeros((0, 2), dtype=np.float64)

        xs = ranges * np.cos(ang)
        ys = ranges * np.sin(ang)
        pts = np.stack([xs, ys], axis=1)

        step = max(1, int(self.downsample_step))
        return pts[::step, :]

    def publish(self, stamp: Time):
        od = Odometry()
        od.header.stamp = stamp.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.child_frame

        od.pose.pose.position.x = float(self.x)
        od.pose.pose.position.y = float(self.y)
        od.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.pub_odom.publish(od)

        tfm = TransformStamped()
        tfm.header.stamp = stamp.to_msg()
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = self.child_frame
        tfm.transform.translation.x = float(self.x)
        tfm.transform.translation.y = float(self.y)
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf_br.sendTransform(tfm)

        ps = PoseStamped()
        ps.header.stamp = stamp.to_msg()
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp.to_msg()
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.path_max_len:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_len :]
        self.pub_path.publish(self.path_msg)

    def _run_icp_two_stage(self, src, dst, guess_rel, extra_yaw=0.0, reacquire_boost=False):
        # tweak guess yaw
        g = se2_compose(guess_rel, (0.0, 0.0, extra_yaw))
        R0, t0 = se2_to_rt(*g)

        max_corr_large = self.max_corr_large * (1.25 if reacquire_boost else 1.0)
        R1, t1, rmse1, pairs1 = icp_2d(
            src, dst, R0, t0,
            iters=self.iters_coarse,
            max_corr=max_corr_large
        )
        R2, t2, rmse2, pairs2 = icp_2d(
            src, dst, R1, t1,
            iters=self.icp_iters,
            max_corr=self.max_corr_dist
        )
        return R2, t2, rmse2, pairs2

    def on_scan(self, msg: LaserScan):
        t_scan = Time.from_msg(msg.header.stamp)

        # time jump / loop
        if self.prev_scan_t is not None:
            dt_raw = (t_scan.nanoseconds - self.prev_scan_t.nanoseconds) * 1e-9
            if self.reset_on_time_jump and (dt_raw < -0.05 or dt_raw > self.max_dt_gap):
                ekf_pose, _ = self._find_ekf_pose_near(t_scan)
                self.reset_state(f"time jump dt={dt_raw:.3f}s", ekf_pose=ekf_pose)

        pts = self.scan_to_points(msg)
        if pts.shape[0] < 20:
            if self._throttle("few_pts", 2.0):
                self.get_logger().warn("Too few valid scan points, skip")
            return

        ekf_pose, best_dt = self._find_ekf_pose_near(t_scan)
        if ekf_pose is None:
            if self._throttle("no_ekf", 2.0):
                self.get_logger().warn("No EKF pose close to scan stamp (sync tolerance too strict?)")
            if self.prev_scan_pts is None:
                self.prev_scan_pts = pts
                self.prev_scan_t = t_scan
            return

        if not self.initialized:
            self.x, self.y, self.yaw = ekf_pose
            self.initialized = True
            self.prev_scan_pts = pts
            self.prev_scan_t = t_scan
            self.prev_ekf_pose = ekf_pose
            self.publish(t_scan)
            return

        if self.prev_scan_pts is None or self.prev_scan_t is None or self.prev_ekf_pose is None:
            self.prev_scan_pts = pts
            self.prev_scan_t = t_scan
            self.prev_ekf_pose = ekf_pose
            return

        dt = (t_scan.nanoseconds - self.prev_scan_t.nanoseconds) * 1e-9
        if dt <= 1e-3:
            self.prev_scan_pts = pts
            self.prev_scan_t = t_scan
            self.prev_ekf_pose = ekf_pose
            return

        # EKF motion prev->curr
        move_ekf = se2_between(self.prev_ekf_pose, ekf_pose)

        # guess mapping(prev_scan->curr_scan) = inv(move_ekf)
        guess_rel = se2_inverse(move_ekf)

        # if we are already rejecting a lot, enable reacquire boost
        reacquire_boost = (self.reject_streak >= 5)

        # yaw search candidates
        ys = [0.0]
        for k in range(1, max(1, self.yaw_search_levels) + 1):
            ys.append(+k * self.yaw_search_step)
            ys.append(-k * self.yaw_search_step)

        best = None
        src = self.prev_scan_pts
        dst = pts

        for yoff in ys:
            R, t, rmse, pairs = self._run_icp_two_stage(src, dst, guess_rel, extra_yaw=yoff, reacquire_boost=reacquire_boost)
            if not math.isfinite(rmse):
                continue
            if best is None or rmse < best[0]:
                best = (rmse, pairs, R, t, yoff)

        if best is None:
            self.reject_streak += 1
            if self._throttle("rej0", 1.0):
                self.get_logger().warn("Reject ICP -> no valid solution")
            if self.reject_streak >= self.reject_reset_N:
                self.reset_state(f"reject streak {self.reject_streak}", ekf_pose=ekf_pose)
            # publish EKF-only update (keep running)
            self.x, self.y, self.yaw = se2_compose((self.x, self.y, self.yaw), move_ekf)
            self.publish(t_scan)
            self.prev_scan_pts = pts
            self.prev_scan_t = t_scan
            self.prev_ekf_pose = ekf_pose
            return

        rmse, pairs, R, t, yoff = best

        # mapping(prev->curr) => robot motion(prev->curr)
        rel = rt_to_se2(R, t)
        move_icp = se2_inverse(rel)

        # inlier ratio
        inlier_ratio = float(pairs) / float(max(1, src.shape[0]))

        # quality gate (ratio+rmse)  << แก้หลัก ๆ อยู่ตรงนี้
        icp_good = (math.isfinite(rmse) and rmse <= self.rmse_max and (pairs >= self.min_inliers or inlier_ratio >= self.min_inlier_ratio))

        # consistency gate vs EKF (กัน ICP หลอก)
        diff = se2_between(move_ekf, move_icp)
        dtrans = math.hypot(diff[0], diff[1])
        drot = abs(diff[2])
        consistent = (dtrans <= self.gate_trans) and (drot <= self.gate_rot)

        if not icp_good or not consistent:
            self.reject_streak += 1
            if self._throttle("rej", 1.0):
                self.get_logger().warn(
                    f"Reject ICP -> EKF only (rmse={rmse:.3f}, pairs={pairs}, ratio={inlier_ratio:.2f}, "
                    f"diff_t={dtrans:.3f}, diff_r={drot:.3f})"
                )
            if self.reject_streak >= self.reject_reset_N:
                self.reset_state(f"reject streak {self.reject_streak}", ekf_pose=ekf_pose)

            # EKF-only update
            self.x, self.y, self.yaw = se2_compose((self.x, self.y, self.yaw), move_ekf)
            self.publish(t_scan)

            self.prev_scan_pts = pts
            self.prev_scan_t = t_scan
            self.prev_ekf_pose = ekf_pose
            return

        # passed
        self.reject_streak = 0

        # residual = inv(move_ekf) ∘ move_icp
        res = se2_between(move_ekf, move_icp)
        dx, dy, dth = res

        # clamp residual (กันกระชาก)
        mag = math.hypot(dx, dy)
        if mag > self.res_clip_trans and mag > 1e-9:
            s = self.res_clip_trans / mag
            dx *= s
            dy *= s
        dth = max(-self.res_clip_rot, min(self.res_clip_rot, dth))

        # alpha schedule (rmse ต่ำ + ratio สูง => ใช้เยอะขึ้น)
        w_rmse = max(0.0, 1.0 - (rmse / self.rmse_max))
        w_ratio = min(1.0, max(0.0, (inlier_ratio - self.min_inlier_ratio) / max(1e-6, (1.0 - self.min_inlier_ratio))))
        a = self.alpha * (0.7 * w_rmse + 0.3 * w_ratio)

        move_fused = se2_compose(move_ekf, (a * dx, a * dy, a * dth))
        self.x, self.y, self.yaw = se2_compose((self.x, self.y, self.yaw), move_fused)

        if self._throttle("ok", 2.0):
            v = math.hypot(move_fused[0], move_fused[1]) / dt
            w = move_fused[2] / dt
            self.get_logger().info(
                f"ICP OK: rmse={rmse:.3f} pairs={pairs} ratio={inlier_ratio:.2f} yaw_off={yoff:+.2f} "
                f"alpha_eff={a:.3f} v={v:.2f} w={w:.2f}"
            )

        self.publish(t_scan)

        self.prev_scan_pts = pts
        self.prev_scan_t = t_scan
        self.prev_ekf_pose = ekf_pose


def main(args=None):
    rclpy.init(args=args)
    node = IcpOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()