#!/usr/bin/env python3
import math
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    pts: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        pts.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return pts


class IcpMapper(Node):
    def __init__(self):
        super().__init__("icp_mapper")

        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom_icp").value
        self.map_topic = self.declare_parameter("map_topic", "/icp_map").value
        self.map_frame = self.declare_parameter("map_frame", "odom").value

        self.resolution = float(self.declare_parameter("resolution", 0.05).value)
        self.width_m = float(self.declare_parameter("width_m", 40.0).value)
        self.height_m = float(self.declare_parameter("height_m", 40.0).value)
        self.origin_x = float(self.declare_parameter("origin_x", -20.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -20.0).value)

        self.min_range = float(self.declare_parameter("min_range", 0.20).value)
        self.max_range = float(self.declare_parameter("max_range", 8.0).value)
        self.beam_step = int(self.declare_parameter("beam_step", 4).value)

        self.lo_occ = float(self.declare_parameter("lo_occ", 0.85).value)
        self.lo_free = float(self.declare_parameter("lo_free", -0.40).value)
        self.lo_min = float(self.declare_parameter("lo_min", -5.0).value)
        self.lo_max = float(self.declare_parameter("lo_max", 5.0).value)

        self.publish_every_n_scans = int(self.declare_parameter("publish_every_n_scans", 2).value)

        self.w = int(round(self.width_m / self.resolution))
        self.h = int(round(self.height_m / self.resolution))

        self.logodds = np.zeros((self.h, self.w), dtype=np.float32)
        self.seen = np.zeros((self.h, self.w), dtype=bool)

        self.last_pose = None  # (x,y,yaw, stamp)
        self.scan_count = 0

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        self.pub_map = self.create_publisher(OccupancyGrid, self.map_topic, 1)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.last_pose = (float(p.x), float(p.y), float(yaw), Time.from_msg(msg.header.stamp))

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def in_bounds(self, cx: int, cy: int) -> bool:
        return (0 <= cx < self.w) and (0 <= cy < self.h)

    def update_ray(self, x0: float, y0: float, x1: float, y1: float):
        c0x, c0y = self.world_to_cell(x0, y0)
        c1x, c1y = self.world_to_cell(x1, y1)
        if not (self.in_bounds(c0x, c0y) and self.in_bounds(c1x, c1y)):
            return

        cells = bresenham(c0x, c0y, c1x, c1y)
        if len(cells) < 2:
            return

        for (cx, cy) in cells[:-1]:
            self.logodds[cy, cx] = float(np.clip(self.logodds[cy, cx] + self.lo_free, self.lo_min, self.lo_max))
            self.seen[cy, cx] = True

        cx, cy = cells[-1]
        self.logodds[cy, cx] = float(np.clip(self.logodds[cy, cx] + self.lo_occ, self.lo_min, self.lo_max))
        self.seen[cy, cx] = True

    def on_scan(self, msg: LaserScan):
        if self.last_pose is None:
            return

        x, y, yaw, _ = self.last_pose
        n = len(msg.ranges)
        step = max(1, int(self.beam_step))

        for i in range(0, n, step):
            r = msg.ranges[i]
            if not math.isfinite(r):
                continue
            if r < self.min_range:
                continue
            r = min(float(r), float(self.max_range), float(msg.range_max))

            ang = msg.angle_min + i * msg.angle_increment
            th = yaw + ang
            xe = x + r * math.cos(th)
            ye = y + r * math.sin(th)
            self.update_ray(x, y, xe, ye)

        self.scan_count += 1
        if self.scan_count % self.publish_every_n_scans == 0:
            self.publish_map(Time.from_msg(msg.header.stamp))

    def publish_map(self, stamp: Time):
        lo = self.logodds
        p = 1.0 - 1.0 / (1.0 + np.exp(lo))
        occ = (p * 100.0).astype(np.int8)
        out = np.where(self.seen, occ, -1).astype(np.int8)

        grid = OccupancyGrid()
        grid.header.stamp = stamp.to_msg()
        grid.header.frame_id = str(self.map_frame)

        info = MapMetaData()
        info.resolution = float(self.resolution)
        info.width = int(self.w)
        info.height = int(self.h)
        info.origin.position.x = float(self.origin_x)
        info.origin.position.y = float(self.origin_y)
        info.origin.orientation.w = 1.0
        grid.info = info

        grid.data = out.reshape(-1).tolist()
        self.pub_map.publish(grid)


def main():
    rclpy.init()
    node = IcpMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()