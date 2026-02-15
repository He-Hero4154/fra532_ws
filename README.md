# FRA532 LAB1 — Wheel Odometry + EKF Yaw Fusion + ICP Odometry + SLAM (ROS 2)

This repo is a **ROS 2 (Humble) colcon workspace** for FRA532 LAB1.
It implements a full odometry stack (wheel → EKF yaw fusion → ICP scan matching) and can run **slam_toolbox** on top.

## What’s inside

- **Wheel odometry** from `/joint_states` → publishes `/odom_wheel`, TF `odom → base_link_wheel`
- **EKF yaw fusion** (state `[x, y, yaw]`):
  - prediction from `/odom_wheel`
  - measurement update from IMU yaw (`/imu`)
  - publishes `/odom_ekf`, TF `odom → base_link_ekf`
- **ICP odometry** (2D point-to-point, coarse-to-fine):
  - aligns consecutive LiDAR scans from `/scan`
  - uses `/odom_ekf` as initial guess + gating (reject bad matches)
  - publishes `/odom_icp`, `/path_icp`, TF `odom → base_link_icp`
- **SLAM (slam_toolbox)**:
  - uses `base_link_icp` as base frame
  - publishes `/map` and SLAM TFs
  - includes a **static TF** `base_link_icp → base_scan` (dataset doesn’t provide `/tf_static`)

## Workspace structure

fra532_ws-main/
├── src/
│   └── lab1_amr/                 # main package (ament_python)
│       ├── lab1_amr/
│       │   ├── wheel_odom.py
│       │   ├── ekf_yaw_fusion.py
│       │   └── icp_odom.py
│       ├── config/
│       │   ├── wheel_params.yaml
│       │   ├── ekf_params.yaml
│       │   ├── icp_params.yaml
│       │   └── slam_params.yaml
│       ├── launch/
│       │   ├── lab1_a_bag.launch.py    # bag play + wheel + ekf + icp (+ rviz)
│       │   ├── slam_full.launch.py     # wheel + ekf + icp + slam_toolbox (+ optional bag)
│       │   └── part3_slam.launch.py    # static TF + slam_toolbox only
│       └── rviz/lab1.rviz
├── data/FRA532_LAB1_DATASET/
│   ├── fibo_floor3_seq00/
│   ├── fibo_floor3_seq01/
│   └── fibo_floor3_seq02/
└── (build/, install/, log/ may exist if previously built)
Requirements

Ubuntu 22.04

ROS 2 Humble

Python 3.10+

System packages
sudo apt update
sudo apt install -y \
  python3-numpy \
  ros-humble-tf2-ros \
  ros-humble-slam-toolbox
Python deps (if your system doesn’t have tf_transformations)

Some environments need this for quaternion_from_euler / euler_from_quaternion:

python3 -m pip install --user tf-transformations
Build
cd fra532_ws-main
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

If you cloned this repo and it already contains build/ install/ log/, you can delete them before building fresh.

Run
Option A — Run Wheel + EKF + ICP and play a bag (recommended quickstart)

This launch file plays the bag and starts wheel_odom, ekf_yaw_fusion, icp_odom, and optionally RViz.

# from workspace root (fra532_ws-main)
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lab1_amr lab1_a_bag.launch.py \
  bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00 \
  rate:=1.0 \
  use_rviz:=true

Switch dataset sequence by changing bag_path:

.../fibo_floor3_seq00

.../fibo_floor3_seq01

.../fibo_floor3_seq02

Option B — Full pipeline + SLAM (slam_toolbox)

slam_full.launch.py starts:

wheel odom

EKF

ICP

static TF (base_link_icp → base_scan)

slam_toolbox

RViz

It can also play the bag if you pass bag_path.

ros2 launch lab1_amr slam_full.launch.py \
  bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00

If you run slam_full.launch.py with bag_path:='' (default), it will NOT play a bag. In that case, run ros2 bag play ... --clock in another terminal.

Option C — Start SLAM only (when wheel/ekf/icp are already running)
ros2 launch lab1_amr part3_slam.launch.py
ROS Interfaces
Topics (published)
Node	Topic	Type
wheel_odom	/odom_wheel	nav_msgs/Odometry
wheel_odom	/path_wheel	nav_msgs/Path
ekf_yaw_fusion	/odom_ekf	nav_msgs/Odometry
ekf_yaw_fusion	/path_ekf	nav_msgs/Path
icp_odom	/odom_icp	nav_msgs/Odometry
icp_odom	/path_icp	nav_msgs/Path
slam_toolbox	/map	nav_msgs/OccupancyGrid
TF frames

odom → base_link_wheel (wheel odom)

odom → base_link_ekf (EKF output)

odom → base_link_icp (ICP output, used as SLAM base)

base_link_icp → base_scan (static TF, required for SLAM)

Parameters (tuning)

All tunables are in src/lab1_amr/config/:

wheel_params.yaml: wheel radius/separation, joint names, frames

ekf_params.yaml: process noise (q_*) and IMU yaw measurement noise (r_yaw), yaw offset behavior

icp_params.yaml: ICP iterations, correspondence limits, gating thresholds, scan filtering (range/FOV)

slam_params.yaml: slam_toolbox basic frames/topic settings (used by part3_slam.launch.py)

Troubleshooting

No movement / time frozen in RViz → make sure the bag is played with --clock and nodes use use_sim_time:=true.

SLAM complains about missing TF to base_scan → use slam_full.launch.py or part3_slam.launch.py (both publish the required static TF).

ICP diverges / jumps → tune icp_params.yaml (especially max_corr_dist, rmse_max, gate_trans, gate_rot) and check LiDAR range/FOV filtering.

Entry points

This package installs these runnable nodes:

ros2 run lab1_amr wheel_odom
ros2 run lab1_amr ekf_yaw_fusion
ros2 run lab1_amr icp_odom 
