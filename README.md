# FRA532 LAB1 — Wheel Odometry + EKF Yaw Fusion + ICP Odometry + SLAM (ROS 2)

This repository is a **ROS 2 (Humble) colcon workspace** for FRA532 LAB1.  
It implements a complete localization pipeline:

**Wheel odometry → EKF (wheel + IMU yaw) → ICP LiDAR odometry → SLAM (slam_toolbox)**

The repo also contains **exported results** (trajectory plots + 2D maps) inside `resultmap_path/`.

---

## Contents / Pipeline Overview

### 1) Wheel Odometry
- Input: `/joint_states`
- Output:
  - `/odom_wheel` (`nav_msgs/Odometry`)
  - `/path_wheel` (`nav_msgs/Path`)
  - TF: `odom → base_link_wheel`

### 2) EKF Yaw Fusion
- State: `[x, y, yaw]`
- Prediction: wheel odom (`/odom_wheel`)
- Measurement update: IMU yaw (`/imu`)
- Output:
  - `/odom_ekf`
  - `/path_ekf`
  - TF: `odom → base_link_ekf`

### 3) ICP Odometry (Scan-to-Scan)
- Input: `/scan`
- Initial guess: from EKF (`/odom_ekf`)
- Output:
  - `/odom_icp`
  - `/path_icp`
  - TF: `odom → base_link_icp`

### 4) SLAM (slam_toolbox)
- Uses ICP as base motion source (`base_link_icp`)
- Output:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - TF chain: `map → odom → base_link_icp`
- A **static TF** `base_link_icp → base_scan` is published because the dataset does not provide `/tf_static`.

### 5) SLAM Pose Output (Trajectory for Report)
Many SLAM stacks publish pose mainly through TF.  
This repo adds a helper node to export SLAM pose as topics:

- `/odom_slam` (`nav_msgs/Odometry`)
- `/path_slam` (`nav_msgs/Path`)

This is the “**SLAM pose output**” required by the lab deliverables.

---

## Repository Structure

```text
fra532_ws/  (or fra532_ws-main/)
├── src/
│   └── lab1_amr/
│       ├── lab1_amr/                 # python nodes
│       │   ├── wheel_odom.py
│       │   ├── ekf_yaw_fusion.py
│       │   ├── icp_odom.py
│       │   ├── icp_mapper.py          # builds /icp_map (2D occupancy grid)
│       │   └── slam_pose_from_tf.py   # builds /odom_slam + /path_slam from TF
│       ├── config/
│       │   ├── wheel_params.yaml
│       │   ├── ekf_params.yaml
│       │   ├── icp_params.yaml
│       │   ├── icp_map_params.yaml
│       │   ├── slam_params.yaml
│       │   └── slam_toolbox_params.yaml
│       ├── launch/
│       │   ├── lab1_a_bag.launch.py
│       │   ├── slam_full.launch.py
│       │   └── part3_slam.launch.py
│       └── rviz/lab1.rviz
├── data/FRA532_LAB1_DATASET/
│   ├── fibo_floor3_seq00/
│   ├── fibo_floor3_seq01/
│   └── fibo_floor3_seq02/
├── resultmap_path/                   # ✅ exported plots + maps for report
└── README.md
```

> Note: `build/`, `install/`, and `log/` are colcon artifacts and should not be committed.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+

### System packages
```bash
sudo apt update
sudo apt install -y   python3-numpy   ros-humble-tf2-ros   ros-humble-slam-toolbox   ros-humble-nav2-map-server
```

### Python deps (if missing)
```bash
python3 -m pip install --user tf-transformations
```

---

## Build

```bash
cd fra532_ws   # or fra532_ws-main
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run

### A) Quickstart: play bag + Wheel + EKF + ICP (and RViz)
```bash
ros2 launch lab1_amr lab1_a_bag.launch.py   bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00   rate:=1.0   use_rviz:=true
```

### B) Full pipeline: Wheel + EKF + ICP + ICP-map + SLAM + SLAM-path (recommended)
```bash
ros2 launch lab1_amr slam_full.launch.py   bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00
```

#### Run SLAM as **closed-loop** (loop closure ON)
```bash
ros2 launch lab1_amr slam_full.launch.py   bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00   do_loop_closing:=true
```

#### Run SLAM as **no-loop** (loop closure OFF)
```bash
ros2 launch lab1_amr slam_full.launch.py   bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00   do_loop_closing:=false
```

#### Disable SLAM (keep ICP mapping only)
```bash
ros2 launch lab1_amr slam_full.launch.py   bag_path:=$(pwd)/data/FRA532_LAB1_DATASET/fibo_floor3_seq00   enable_slam:=false   enable_icp_mapping:=true
```

### C) SLAM only (when wheel/ekf/icp already running)
```bash
ros2 launch lab1_amr part3_slam.launch.py
```

---

## Results Folder: `resultmap_path/` (สำคัญมาก)

`resultmap_path/` contains **exported figures** for the report:

### 1) Trajectory / Path overlay plots
Files:
- `bag0_path.png`
- `bag01_path.png`
- `bag02_path.png`

Naming convention used in this repo:
- `bag0`  ≈ `seq00`
- `bag01` ≈ `seq01`
- `bag02` ≈ `seq02`

These figures overlay all methods in the same plot.

#### ✅ Color Legend (ต้องใส่ในรายงาน)
In **all path plots** inside `resultmap_path/`:

- **Green**  = **Wheel odometry** (`/odom_wheel`)
- **Yellow** = **EKF odometry** (`/odom_ekf`)
- **Blue**   = **ICP odometry** (`/odom_icp`)
- **Red**    = **SLAM path** (`/path_slam` from slam_toolbox TF)

This legend applies to trajectory/path comparisons used in the deliverables.

### 2) 2D maps (Occupancy Grid images)

#### ICP 2D map (dead-reckoning, no global correction)
Files:
- `bag0_2d_icp.png`
- `bag01_2d_icp.png`
- `bag02_2d_icp.png`

These maps are generated by projecting LiDAR scans using **ICP odometry pose** only
(i.e., mapping without loop closure). Expect drift artifacts such as “stretching / double walls”.

#### SLAM 2D map (slam_toolbox)
Files:
- `bag0_2d_slam.png`
- `bag01_2d_slam.png`
- `bag02_2d_slam.png`

These are maps from `slam_toolbox` (`/map`), which is **loop-closure capable**.
If loop closure succeeds, the map should appear more consistent than ICP mapping.

---

## How to Re-generate 2D Maps (Save as files)

### Save SLAM map (`/map`)
```bash
ros2 run nav2_map_server map_saver_cli -f slam_map
```

### Save ICP map (published as `/icp_map`)
```bash
ros2 run nav2_map_server map_saver_cli -f icp_map --ros-args -r map:=/icp_map
```

> Tip: move saved outputs into `resultmap_path/` for report submission.

---

## ROS Interfaces

### Topics (published)

| Method / Node | Topic | Type |
|---|---|---|
| Wheel odom | `/odom_wheel` | `nav_msgs/Odometry` |
| Wheel odom | `/path_wheel` | `nav_msgs/Path` |
| EKF | `/odom_ekf` | `nav_msgs/Odometry` |
| EKF | `/path_ekf` | `nav_msgs/Path` |
| ICP odom | `/odom_icp` | `nav_msgs/Odometry` |
| ICP odom | `/path_icp` | `nav_msgs/Path` |
| ICP mapper | `/icp_map` | `nav_msgs/OccupancyGrid` |
| SLAM (slam_toolbox) | `/map` | `nav_msgs/OccupancyGrid` |
| SLAM pose export | `/odom_slam` | `nav_msgs/Odometry` |
| SLAM pose export | `/path_slam` | `nav_msgs/Path` |

### TF frames
- `odom → base_link_wheel`
- `odom → base_link_ekf`
- `odom → base_link_icp`
- `map → odom` (slam_toolbox)
- `base_link_icp → base_scan` (static TF)

---

## Deliverables Checklist (Lab Submission)

Students should submit:

1) **Source code**
2) **README**
3) **Plots of trajectories** (Wheel, EKF, ICP, SLAM pose output)
4) **Generated 2D maps** (ICP map and slam_toolbox map)
5) **Discussion** comparing:
   - **Accuracy** (qualitative or against SLAM reference)
   - **Drift** (growth over time / loop return mismatch)
   - **Robustness** (failure cases: corridor, low features, dynamic objects, IMU bias, wheel slip)

---

## Notes / Known Limitations

- Wheel odometry drift accumulates quickly (slip, scale error).
- EKF performance depends on Q/R tuning and IMU yaw bias behavior.
- ICP can diverge in repetitive corridors or low-feature scenes; EKF initial guess is critical.
- SLAM map quality depends on loop closure success and parameter tuning.

---

