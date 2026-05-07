## 1. Prerequisites

### 1.1 Ubuntu and ROS

* Ubuntu 20.04 for ROS Noetic (recommended)
* Ubuntu 22.04 for ROS Humble (ROS2, livox_ros_driver2 only)

### 1.2 PCL & Eigen & C++

```bash
sudo apt install libc++-dev libc++abi-dev
sudo apt-get install libeigen3-dev
```

### 1.3 Open3D

Recommend use the precompiled Open3D library we provided. It can be downloaded from [Baidu Netdisk](https://pan.baidu.com/s/1vTLXVYJ6JBlbhNpDf87Cdg?pwd=spdg), ``open3d141.zip``(tested) for x86 architecture and ``open3d141_arm.zip``(not fully tested) for arm architecture.

Open [open3d_loc/CMakeLists.txt](open3d_loc/CMakeLists.txt), replace ``Open3D_DIR`` with the folder where you unzipped open3d141.zip:

```
# For example
set(Open3D_DIR "/home/liar/open3d141/lib/cmake/Open3D")
```

Or build from source following [Open3D documentation/Build from source](https://www.open3d.org/docs/release/compilation.html).

### 1.4 Livox-SDK2

Follow the installation guide in [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

## 2. Build

A pre-built livox_ros_driver workspace must be sourced **before** building this one:

```bash
# 1. Build livox_ros_driver2 in a separate workspace
source /opt/ros/noetic/setup.bash
mkdir -p ~/ws_livox/src && cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ..
catkin_make -DROS_EDITION=ROS1
source ~/ws_livox/devel/setup.bash

# 2. Build this workspace
mkdir -p ~/ws_loc/src && cd ~/ws_loc/src
git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git
cd ..
catkin_make -DROS_EDITION=ROS1
```

> **Note:** `-DROS_EDITION=ROS1` is required; livox_ros_driver2 won't compile without it.

## 3. Parameter Configuration

### 3.1 livox_ros_driver2

Modify your computer IP address following [Unitree SDK Development Guide/Lidar Route](https://support.unitree.com/home/zh/G1_developer/lidar_Instructions).

Extrinsics of pointcloud and IMU are configured via ``extrinsic_parameter`` in [livox_ros_driver2/config/MID360_config.json](livox_ros_driver2/config/MID360_config.json):

| Robot | roll | pitch | yaw |
|-------|------|-------|-----|
| GO2   | 0.0  | 13.0  | 0.0 |
| G1    | 180.0| 0.0   | 0.0 |

```
"extrinsic_parameter" : {
  "roll": 0.0,
  "pitch": 13.0,
  "yaw": 0.0,
  "x": 0,
  "y": 0,
  "z": 0
}
```

For other parameters (such as IP), follow [livox_ros_driver2](livox_ros_driver2/README.md).

### 3.2 fast_lio

Config files per robot variant:

| Robot | Config |
|-------|--------|
| GO2   | [FAST_LIO/config/mid360_go2.yaml](FAST_LIO/config/mid360_go2.yaml) |
| G1    | [FAST_LIO/config/mid360_g1.yaml](FAST_LIO/config/mid360_g1.yaml) |

> **Important:** `extrinsic_T` / `extrinsic_R` in these configs are the **Mid-360 internal IMU-to-lidar offset**, NOT the sensor-to-robot mounting offset. The mounting offset is handled by livox_ros_driver2's `extrinsic_parameter` (rotation) and a static TF publisher (translation).

### 3.3 open3d_loc

#### 3.3.1 Pointcloud Map

Place your pointcloud map in the [data](data) folder and set ``path_map`` in the launch file. Both ``.pcd`` and ``.ply`` formats are accepted.

**Single-map mode (G1 example):**
```
<param name="path_map" type="string" value="$(find open3d_loc)/../data/map.ply" />
```

**Multi-map mode (GO2 example):** See Section 3.3.2 below. When `maps` list is configured, `path_map` is ignored.

#### 3.3.2 Multi-Map Switching

The system supports automatic switching between multiple independent pointcloud maps, each with its own coordinate frame.

Configure via the `maps` and `map_switch` parameters in [open3d_loc/launch/open3d_loc_go2.launch](open3d_loc/launch/open3d_loc_go2.launch):

```yaml
maps:
  - { name: mapA, path: $(find open3d_loc)/../data/mapA.ply,
      switch_zone: [xmin, ymin, xmax, ymax],
      next_initialpose: [x, y, z, roll, pitch, yaw_deg] }
  - { name: mapB, path: $(find open3d_loc)/../data/mapB.ply,
      initialpose: [x, y, z, roll, pitch, yaw_deg] }

map_switch:
  enable: true
  active_map: mapA
  bbox_shrink: 1.0
  verify_fitness: 0.6
  verify_consecutive: 3
  verify_period_ms: 500
  cooldown_ms: 5000
  verify_icp_threshold: 0.3
  verify_max_translation: 1.0
  verify_max_yaw_deg: 20.0
  post_switch_min_fitness: 0.3
  allow_uncalibrated_switch: true
```

**Key concepts:**

| Parameter | Description |
|-----------|-------------|
| `name` | Map identifier. Each map gets an independent frame `map_<name>`, e.g. `map_mapA` |
| `switch_zone` | AABB `[xmin, ymin, xmax, ymax]` in current active map frame that triggers verification of the next map |
| `initialpose` | Initial pose as `T_map_base` [x, y, z, roll, pitch, yaw_deg]: robot's base_link in this map's coordinate frame |
| `next_initialpose` | When switching from this map, the expected `T_map_base` in the next map's coordinate frame |
| `verify_fitness` | Background ICP fitness threshold to count as one successful verification |
| `verify_consecutive` | Number of consecutive successful verifications before attempting switch |
| `post_switch_min_fitness` | Precommit fitness check; switch is rejected if below this value |

> **Note:** `initialpose` and `next_initialpose` represent `T_map_base` (robot in map frame), NOT `T_map_odom`. The system automatically converts to `T_map_odom = T_map_base * inv(T_odom_base)` at runtime.

See [open3d_loc/config/maps_example.yaml](open3d_loc/config/maps_example.yaml) for a template.

#### 3.3.3 Fitness Thresholds

``threshold_fitness_init`` is the initial pointcloud registration threshold, ``threshold_fitness`` is the regular registration threshold. These may need tuning per scenario.

#### 3.3.4 IMU-to-base_link TF

| Robot | Translation (imu_link in base_link frame) |
|-------|-------------------------------------------|
| GO2   | `[0.1870, 0, 0.0803]` |
| G1    | Identity (zero translation) |

This is set via static TF publishers in the launch files.

## 4. Run

### 4.0 Pointcloud Map

We recommend [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) and [Point-LIO](https://github.com/hku-mars/Point-LIO) for small-scale mapping.

Use [CloudCompare](https://github.com/CloudCompare/CloudCompare) to fine-tune pointcloud maps: align the ground plane, downsample, and crop out artifacts.

### 4.1 Localization (open3d_loc + fast_lio)

Two robot variants — use the matching launch files:

| Robot | Command |
|-------|---------|
| GO2   | `roslaunch open3d_loc localization_3d_go2.launch` |
| G1    | `roslaunch open3d_loc localization_3d_g1.launch`  |

Each `localization_3d_*.launch` starts both `fast_lio` + `open3d_loc` + rviz.

rviz of fast_lio is **disabled by default** to save memory:
```
<arg name="rviz" default="false" />
```

### 4.2 Initial Pose

In **rviz**, click **2D Pose Estimate**, and set a position and orientation on the pointcloud map via the **green arrow**.

<div align="center">
<img src="doc/initial_pose.jpg" width=75% />
</div>

### 4.3 Livox Driver

In a **second terminal**:

```bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 4.4 Rosbag Playback

No need to launch livox_ros_driver2 when using rosbag. However, the rosbag must have been recorded using our **modified livox_ros_driver2**.

Set ``use_sim_time`` to `"true"` in both `mapping_mid360_*.launch` and `open3d_loc_*.launch`, then:

```bash
rosbag play xxx.bag --clock
```

> GO2 launch files have `use_sim_time` set to `true` by default; G1 uses `false`.

### 4.5 Check by rqt_graph

```
rqt_graph
```

<div align="center">
<img src="doc/rqt_graph.jpg" width=85% />
</div>

## 5. Architecture / Data Flow

```
livox_ros_driver2  ──/livox/lidar, /livox/imu──►  fast_lio
                                                    │
                        /Odometry_loc ──────────────┤
                        /cloud_registered_1 ────────┤
                                                    ▼
                                              open3d_loc (ICP against offline map)
                                                    │
                        /localization_3d ◄──────────┘
                        /localization_3d_confidence
                        /localization_3d_delay_ms
                        /localization_3d_active_map
                        /localization_3d_status
```

Key subscribed topics in `global_localization_node`: `/Odometry_loc` (nav_msgs/Odometry), `/cloud_registered_1` (sensor_msgs/PointCloud2), `/initialpose`.

TF tree:
```
localization_world          (display frame, for RViz visual continuity)
  └── map_<active>          (active map frame, e.g. map_vsis515)
      └── odom
          └── base_link
              └── imu_link
```

## 6. Result Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/localization_3d` | geometry_msgs/PoseStamped | Localization result in active map frame |
| `/localization_3d_confidence` | std_msgs/Float32 | ICP registration fitness; higher = more accurate |
| `/localization_3d_delay_ms` | std_msgs/Float32 | Delay between localization result and odometry (ms) |
| `/localization_3d_active_map` | std_msgs/Float32 | Current active map name (latched) |
| `/localization_3d_status` | std_msgs/String | Status: `INITIALIZING`, `OK`, `VERIFYING:<map>`, `PRECOMMIT:<map>`, `SWITCHED:<old>-><new>`, `NEED_CALIBRATION:<map>`, `MANUAL_INITIALPOSE`, `REJECTED` |

```bash
rostopic echo /localization_3d
rostopic echo /localization_3d_active_map
rostopic echo /localization_3d_status
```

## 7. Demo

### 7.0 Rosbag for Demo

- Download from [Baidu Netdisk](https://pan.baidu.com/s/1vTLXVYJ6JBlbhNpDf87Cdg?pwd=spdg) (pwd: `spdg`). `mapping.bag` for mapping, `loc.bag` for localization.
- Color pointcloud map demo: [Baidu Netdisk](https://pan.baidu.com/s/1lN0ZjEEJDp8-3oz8JRtt7w?pwd=6bqu) (pwd: `6bqu`)
- If you have an iPhone with LiDAR, try the **3D Scanner** app. See [doc/3DScanner.mp4](doc/3DScanner.mp4).

### 7.1 Mapping

Use **fast_lio** for mapping. See [doc/demo_mapping_fastlio.mp4](doc/demo_mapping_fastlio.mp4).

### 7.2 Fine Tune (Recommended)

Use **CloudCompare** to downsample, align the ground plane (ensure z≈0), and crop artifacts. See [doc/demo_finetune.mp4](doc/demo_finetune.mp4).

### 7.3 Localization

See [doc/demo_loc.mp4](doc/demo_loc.mp4).

## 8. Acknowledgments

The odometry module is based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), and the localization module refers to [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION). Thanks for their great work.
