# GO2 实机运行修改说明

本文用于指导另一个代码助手按步骤修改本仓库，使定位系统更适合直接部署到真实 Unitree GO2 + Livox MID360 环境。

当前仓库已经具备 GO2 定位链路：`livox_ros_driver2` 发布 `/livox/lidar`、`/livox/imu`，`FAST_LIO` 发布 `/Odometry_loc`、`/cloud_registered_1`，`open3d_loc` 基于离线点云地图输出 `/localization_3d` 以及 TF。

但是当前配置明显偏向 rosbag/实验数据，不建议不改配置直接上真实 GO2。

## 0. 当前系统是否会修改 GO2 本体

当前仓库没有发现 Unitree 运动控制接口，也没有发布 `/cmd_vel`、`lowcmd`、`sport` 等控制命令。

运行这些节点本身不会控制 GO2 行走，也不会修改 GO2 底层控制系统。它主要做三件事：

1. 从 MID360 读取 LiDAR/IMU 数据。
2. 在 ROS 中发布 FAST-LIO 里程计、配准点云、定位结果和 TF。
3. 根据 `livox_ros_driver2/config/MID360_config.json` 向 Livox 驱动提供网络、数据格式、外参参数。

需要注意：如果实机上已经有其他节点发布同名 TF 或同名 topic，当前 launch 可能造成 TF 冲突或 topic 冲突。当前仓库不做底盘控制，但它会在 ROS 图里发布 `base_link`、`motion_link`、`odom`、`camera_init`、`map` 等相关 TF/topic。

## 1. 当前仓库的实机风险点

### 1.1 GO2 launch 默认启用了仿真时间

文件：

- `FAST_LIO/launch/mapping_mid360_go2.launch`
- `open3d_loc/launch/open3d_loc_go2.launch`

当前 GO2 两个 launch 都写死了：

```xml
<param name="use_sim_time" value="true" />
```

真实 GO2 实机没有 `/clock` 时，`use_sim_time=true` 会导致依赖 ROS time 的逻辑异常，常见表现是等待、计时、延迟统计或可视化不正常。

必须改为：实机默认 `false`，rosbag 回放时通过 launch 参数显式传 `true`。

### 1.2 GO2 默认加载了实验室多地图

文件：

- `open3d_loc/launch/open3d_loc_go2.launch`

当前 `path_map` 被注释掉，实际启用的是：

```yaml
maps:
- { name: vsis515, path: $(find open3d_loc)/data/vsis515.pcd, ... }
- { name: vsis516, path: $(find open3d_loc)/data/vsis516.pcd, ... }
map_switch:
  enable: true
```

这不是通用 GO2 实机配置。真实场景应该默认使用用户提供的单地图，只有在确实完成多地图标定时才启用多地图切换。

### 1.3 多地图切换允许未标定切换

当前 GO2 多地图参数里有：

```yaml
allow_uncalibrated_switch: true
```

同时示例 `next_initialpose` 是全零。这对真实环境有风险，可能在错误初值下尝试切换地图。

实机默认应改为：

```yaml
allow_uncalibrated_switch: false
```

并要求每个切换入口都提供真实标定过的 `next_initialpose`。

### 1.4 GO2 FAST-LIO 默认保存 PCD

文件：

- `FAST_LIO/config/mid360_go2.yaml`

当前：

```yaml
pcd_save:
    pcd_save_en: true
    interval: -1
```

`interval: -1` 表示所有帧存成一个 PCD。实机长时间定位时可能持续占内存/磁盘，不适合作为定位默认配置。

实机定位默认应改为：

```yaml
pcd_save:
    pcd_save_en: false
    interval: -1
```

如果要建图或录制点云，再用单独 launch 或手动改为 `true`。

### 1.5 Open3D 路径硬编码

文件：

- `open3d_loc/CMakeLists.txt`

当前写死：

```cmake
set(Open3D_DIR "/root/loc_ws/lib/open3d141/lib/cmake/Open3D")
```

这会覆盖用户通过 `catkin_make -DOpen3D_DIR=...` 传入的路径。实机电脑路径不同就会编译失败。

应该改成“只有用户没传 `Open3D_DIR` 时才使用默认值”。

### 1.6 Livox 网络 IP 是硬编码的

文件：

- `livox_ros_driver2/config/MID360_config.json`

当前 host IP 是：

```json
"cmd_data_ip" : "192.168.123.222",
"push_msg_ip": "192.168.123.222",
"point_data_ip": "192.168.123.222",
"imu_data_ip" : "192.168.123.222"
```

LiDAR IP 是：

```json
"ip" : "192.168.123.120"
```

这些值必须和实机网络一致。Claude 不应随意猜测 IP，只能把配置做清楚，或者新增模板和说明。

### 1.7 RViz 在总 launch 中无条件启动

文件：

- `open3d_loc/launch/localization_3d_go2.launch`

当前总启动文件无条件启动 RViz：

```xml
<node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz_map_cur" args="-d $(find open3d_loc)/rviz_cfg/loc_map_cur.rviz" />
```

真实机器人上如果是无显示器部署，RViz 会失败或浪费资源。应该加 `rviz` 参数控制。

## 2. 必须修改项

### 2.1 修改 `FAST_LIO/launch/mapping_mid360_go2.launch`

目标：GO2 FAST-LIO 实机默认不用仿真时间，但保留 rosbag 回放能力。

建议修改：

```xml
<launch>
<!-- Launch file for Livox MID360 LiDAR on Unitree Go2 -->

    <arg name="rviz" default="false" />
    <arg name="use_sim_time" default="false" />

    <rosparam command="load" file="$(find fast_lio)/config/mid360_go2.yaml" />

    <param name="feature_extract_enable" type="bool" value="0"/>
    <param name="point_filter_num" type="int" value="3"/>
    <param name="max_iteration" type="int" value="3" />
    <param name="filter_size_surf" type="double" value="0.5" />
    <param name="filter_size_map" type="double" value="0.5" />
    <param name="cube_side_length" type="double" value="1000" />
    <param name="runtime_pos_log_enable" type="bool" value="0" />

    <node pkg="fast_lio" type="fastlio_mapping" name="fast_lio_node" output="screen" />

    <param name="use_sim_time" value="$(arg use_sim_time)" />

    <group if="$(arg rviz)">
        <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find fast_lio)/rviz_cfg/loam_livox.rviz" />
    </group>
</launch>
```

验收点：

- `roslaunch fast_lio mapping_mid360_go2.launch` 后，`rosparam get /use_sim_time` 应是 `false`。
- `roslaunch fast_lio mapping_mid360_go2.launch use_sim_time:=true` 后，`rosparam get /use_sim_time` 应是 `true`。

### 2.2 修改 `open3d_loc/launch/open3d_loc_go2.launch`

目标：GO2 open3d 定位实机默认使用单地图；多地图作为可选模式；仿真时间默认关闭。

建议新增 launch 参数：

```xml
<arg name="use_sim_time" default="false" />
<arg name="map_path" default="$(find open3d_loc)/../data/map.ply" />
<arg name="use_multimap" default="false" />
<arg name="maps_config" default="$(find open3d_loc)/config/maps_go2_example.yaml" />
```

把当前写死的 `maps` 和 `map_switch` 从 launch 里移除或注释掉，改成：

```xml
<!-- 单地图模式：实机默认。 -->
<group unless="$(arg use_multimap)">
    <param name="path_map" type="string" value="$(arg map_path)" />
</group>

<!-- 多地图模式：只有显式 use_multimap:=true 时加载。 -->
<group if="$(arg use_multimap)">
    <rosparam command="load" file="$(arg maps_config)" subst_value="true" />
</group>
```

文件末尾把：

```xml
<param name="use_sim_time" value="true" />
```

改成：

```xml
<param name="use_sim_time" value="$(arg use_sim_time)" />
```

注意：

- 单地图模式只能设置 `path_map`，不能同时加载 `maps`。
- 多地图模式加载的 YAML 必须包含 `maps:` 和 `map_switch:`。
- `maps` 一旦存在，代码会忽略 `path_map`。

验收点：

- 默认启动时，参数里应该有 `global_localization_node/path_map`。
- 默认启动时，不应该有实验室地图 `vsis515`、`vsis516` 被加载。
- `use_multimap:=true maps_config:=...` 时才加载 `maps`。

### 2.3 修改 `open3d_loc/launch/localization_3d_go2.launch`

目标：总启动文件把参数传给 FAST-LIO 和 open3d_loc，并让 RViz 可开关。

建议改成类似：

```xml
<launch>
    <arg name="use_sim_time" default="false" />
    <arg name="rviz" default="false" />
    <arg name="map_path" default="$(find open3d_loc)/../data/map.ply" />
    <arg name="use_multimap" default="false" />
    <arg name="maps_config" default="$(find open3d_loc)/config/maps_go2_example.yaml" />

    <include file="$(find fast_lio)/launch/mapping_mid360_go2.launch">
        <arg name="use_sim_time" value="$(arg use_sim_time)" />
        <arg name="rviz" value="false" />
    </include>

    <include file="$(find open3d_loc)/launch/open3d_loc_go2.launch">
        <arg name="use_sim_time" value="$(arg use_sim_time)" />
        <arg name="map_path" value="$(arg map_path)" />
        <arg name="use_multimap" value="$(arg use_multimap)" />
        <arg name="maps_config" value="$(arg maps_config)" />
    </include>

    <group if="$(arg rviz)">
        <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz_map_cur" args="-d $(find open3d_loc)/rviz_cfg/loc_map_cur.rviz" />
    </group>
</launch>
```

实机运行示例：

```bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/absolute/path/to/go2_map.pcd rviz:=false
```

rosbag 回放示例：

```bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/absolute/path/to/go2_map.pcd use_sim_time:=true rviz:=true
rosbag play xxx.bag --clock
```

### 2.4 修改 `FAST_LIO/config/mid360_go2.yaml`

目标：实机定位默认不保存完整 PCD，避免长时间运行占用磁盘/内存。

把：

```yaml
pcd_save:
    pcd_save_en: true
    interval: -1
```

改为：

```yaml
pcd_save:
    pcd_save_en: false
    interval: -1
```

不要改：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"

preprocess:
    lidar_type: 1
```

原因：当前 FAST-LIO 在 `lidar_type == AVIA` 时订阅 `livox_ros_driver2/CustomMsg`，而 `livox_ros_driver2/launch_ROS1/msg_MID360.launch` 的 `xfer_format` 默认是 `1`，两者是匹配的。

### 2.5 修改 `open3d_loc/CMakeLists.txt`

目标：不要无条件覆盖用户传入的 `Open3D_DIR`。

把当前：

```cmake
set(Open3D_DIR "/root/loc_ws/lib/open3d141/lib/cmake/Open3D")

find_package(Open3D REQUIRED)
```

改成：

```cmake
if(NOT Open3D_DIR)
  if(DEFINED ENV{Open3D_DIR})
    set(Open3D_DIR "$ENV{Open3D_DIR}" CACHE PATH "Path to Open3DConfig.cmake")
  else()
    set(Open3D_DIR "/root/loc_ws/lib/open3d141/lib/cmake/Open3D" CACHE PATH "Path to Open3DConfig.cmake")
  endif()
endif()

find_package(Open3D REQUIRED)
```

这样用户可以：

```bash
catkin_make -DROS_EDITION=ROS1 -DOpen3D_DIR=/real/path/to/open3d/lib/cmake/Open3D
```

或者：

```bash
export Open3D_DIR=/real/path/to/open3d/lib/cmake/Open3D
catkin_make -DROS_EDITION=ROS1
```

### 2.6 新增 `open3d_loc/config/maps_go2_example.yaml`

目标：把多地图配置变成显式可选模板，不要默认污染实机单地图启动。

建议新增文件：

```yaml
# 多地图配置模板。只有 roslaunch 时传 use_multimap:=true 才应加载本文件。
# initialpose / next_initialpose 均表示 T_map_base:
# [x, y, z, roll_deg, pitch_deg, yaw_deg]

maps:
  - name: map_a
    path: $(find open3d_loc)/../data/map_a.pcd
    switch_zone: [0.0, 0.0, 1.0, 1.0]
    next_initialpose: [0, 0, 0, 0, 0, 0]

  - name: map_b
    path: $(find open3d_loc)/../data/map_b.pcd
    initialpose: [0, 0, 0, 0, 0, 0]

map_switch:
  enable: true
  active_map: map_a
  bbox_shrink: 1.0
  verify_fitness: 0.6
  verify_consecutive: 3
  verify_period_ms: 500
  cooldown_ms: 5000
  verify_icp_threshold: 0.3
  verify_max_translation: 1.0
  verify_max_yaw_deg: 20.0
  post_switch_min_fitness: 0.3
  allow_uncalibrated_switch: false
```

重要：这个文件只是模板。实机不能用全零 `next_initialpose` 做自动切图。全零表示没有标定好。

## 3. 必须人工确认的实机参数

这些值不能由 Claude 硬猜，必须由用户根据真实机器人填写。

### 3.1 MID360 和主机 IP

文件：

- `livox_ros_driver2/config/MID360_config.json`

确认：

- 主机 LiDAR 网卡 IP。
- MID360 IP。
- GO2 和上位机之间是否同一网段。
- 防火墙是否允许 UDP 端口 56100 到 56501。

需要修改所有 host IP 字段：

```json
"cmd_data_ip" : "<HOST_LIDAR_NIC_IP>",
"push_msg_ip": "<HOST_LIDAR_NIC_IP>",
"point_data_ip": "<HOST_LIDAR_NIC_IP>",
"imu_data_ip" : "<HOST_LIDAR_NIC_IP>"
```

需要确认 LiDAR IP：

```json
"ip" : "<MID360_IP>"
```

修改后验收：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 3.2 MID360 安装旋转外参

文件：

- `livox_ros_driver2/config/MID360_config.json`

当前 GO2 假设：

```json
"extrinsic_parameter" : {
  "roll": 0.0,
  "pitch": 13.0,
  "yaw": 0.0,
  "x": 0,
  "y": 0,
  "z": 0
}
```

如果实际 MID360 安装角度就是前倾 13 度，保留。否则必须改成真实安装角。

不要把平移外参同时写进这里和 TF。当前系统设计是：

- Livox driver 的 `extrinsic_parameter` 只处理安装旋转。
- `open3d_loc/launch/open3d_loc_go2.launch` 的静态 TF 处理 `base_link -> imu_link` 平移。
- `FAST_LIO/config/mid360_go2.yaml` 的 `extrinsic_T` / `extrinsic_R` 是 MID360 传感器内部 IMU 和 LiDAR 的偏移，不是机器人安装外参。

### 3.3 GO2 `base_link -> imu_link` 平移

文件：

- `open3d_loc/launch/open3d_loc_go2.launch`

当前写死：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="imulink2baselink"
      args="0.1870 0 0.0803 0 0 0 1 base_link imu_link" />
```

含义：`imu_link` 在 `base_link` 下的位置是：

```text
x = 0.1870 m
y = 0
z = 0.0803 m
```

如果雷达安装位置和这个值不同，必须改。

建议 Claude 把这几个值参数化，避免以后反复改 launch：

```xml
<arg name="base_to_imu_x" default="0.1870" />
<arg name="base_to_imu_y" default="0.0" />
<arg name="base_to_imu_z" default="0.0803" />

<node pkg="tf2_ros" type="static_transform_publisher" name="imulink2baselink"
      args="$(arg base_to_imu_x) $(arg base_to_imu_y) $(arg base_to_imu_z) 0 0 0 1 base_link imu_link" />
```

如果 GO2 原本已有 `base_link -> imu_link` 或类似 TF，不能重复发布。需要只保留一个 TF 源。

### 3.4 点云地图路径

真实环境必须提供现场点云地图。当前默认地图不一定对应现场。

建议运行时传绝对路径：

```bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/home/unitree/maps/site_a.pcd
```

地图要求：

- `.pcd` 或 `.ply`。
- 地面方向和机器人运行坐标一致。
- 和实时 LiDAR 点云尺度一致，单位是米。
- 尽量裁掉动态物体、墙外杂点、过远噪声。

### 3.5 初始化位姿

`global_localization_node` 启动后会等待：

- `/Odometry_loc`
- `/cloud_registered_1`

然后执行 ICP 初始化。初始化需要连续 2 次 fitness 超过 `threshold_fitness_init` 才成功。

如果初值很差，初始化会失败或耗时很长。建议：

- 单地图时在 `open3d_loc/config/loc_param_go2.yaml` 或 launch 中填写 `initialpose`。
- 或在 RViz 里用 `2D Pose Estimate` 发布 `/initialpose`。

`initialpose` 含义是：

```text
T_map_base = [x, y, z, roll_deg, pitch_deg, yaw_deg]
```

不是 `T_map_odom`。

## 4. 建议修改项

### 4.1 参数化 topic 名称

当前源码硬编码：

- `FAST_LIO` 订阅 `/livox/lidar`、`/livox/imu`
- `FAST_LIO` 发布 `/Odometry_loc`、`/cloud_registered_1`
- `open3d_loc` 订阅 `/Odometry_loc`、`/cloud_registered_1`、`/initialpose`

如果真实 GO2 上不会有同名冲突，可以先不改。

如果要做成更工程化部署，建议 Claude 后续把 `open3d_loc/src/global_localization.cpp` 中这些 topic 改为私有参数：

```xml
<param name="odom_topic" value="/Odometry_loc" />
<param name="scan_topic" value="/cloud_registered_1" />
<param name="initialpose_topic" value="/initialpose" />
```

并保持默认值不变，避免破坏现有 launch。

### 4.2 参数化 frame 名称

当前代码和 launch 使用：

- `base_link`
- `motion_link`
- `imu_link`
- `odom`
- `camera_init`
- `localization_world`

如果 GO2 的上层导航使用别的 frame，需要统一修改，不要只改 launch。

建议优先保持当前 frame 名，先跑通定位。只有接入已有导航栈时再做 frame 参数化。

### 4.3 保留 G1 行为

本次目标是 GO2 实机配置。Claude 修改时不要改坏：

- `open3d_loc/launch/localization_3d_g1.launch`
- `open3d_loc/launch/open3d_loc_g1.launch`
- `FAST_LIO/launch/mapping_mid360_g1.launch`
- `FAST_LIO/config/mid360_g1.yaml`

除非明确需要同步通用能力，比如 `Open3D_DIR`。

## 5. 不要修改的点

### 5.1 不要把 GO2 安装平移写进 FAST-LIO extrinsic_T

文件：

- `FAST_LIO/config/mid360_go2.yaml`

当前：

```yaml
extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
extrinsic_R: [ 1, 0, 0,
               0, 1, 0,
               0, 0, 1]
```

这描述的是 MID360 内部 LiDAR 和 IMU 的偏移，不是 MID360 到 GO2 机身的安装偏移。

GO2 安装偏移应由：

- `livox_ros_driver2/config/MID360_config.json` 的旋转外参。
- `open3d_loc/launch/open3d_loc_go2.launch` 的静态 TF 平移。

共同处理。

### 5.2 不要把 `msg_MID360.launch` 的 `xfer_format` 改成 0

文件：

- `livox_ros_driver2/launch_ROS1/msg_MID360.launch`

当前：

```xml
<arg name="xfer_format" default="1"/>
```

保持 `1`。原因：FAST-LIO 当前 `lidar_type: 1` 时订阅的是 `livox_ros_driver2/CustomMsg`，需要 Livox custom format。

如果改成 `0`，驱动会发布 `sensor_msgs/PointCloud2`，当前 FAST-LIO 订阅回调就不匹配。

### 5.3 不要默认启用未标定多地图切换

真实机器人上不要默认：

```yaml
allow_uncalibrated_switch: true
```

也不要用全零 `next_initialpose` 做自动切图。

## 6. 实机启动顺序

### 6.1 编译

必须先 source 已编译的 Livox 依赖环境，再编译本仓库：

```bash
source /opt/ros/noetic/setup.bash
source ~/ws_livox/devel/setup.bash
cd /root/loc_ws
catkin_make -DROS_EDITION=ROS1 -DOpen3D_DIR=/real/path/to/open3d/lib/cmake/Open3D
```

如果 `Open3D_DIR` 已通过环境变量设置：

```bash
export Open3D_DIR=/real/path/to/open3d/lib/cmake/Open3D
catkin_make -DROS_EDITION=ROS1
```

### 6.2 启动 Livox 驱动

终端 1：

```bash
source /root/loc_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

验收：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /livox/imu/header
```

### 6.3 启动 GO2 定位

终端 2：

```bash
source /root/loc_ws/devel/setup.bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/absolute/path/to/go2_map.pcd rviz:=false
```

验收：

```bash
rosparam get /use_sim_time
rostopic hz /Odometry_loc
rostopic hz /cloud_registered_1
rostopic echo /localization_3d_confidence
rostopic echo /localization_3d_delay_ms
```

`rosparam get /use_sim_time` 实机必须是：

```text
false
```

### 6.4 初始化

如果没有提前设置 `initialpose`，打开 RViz：

```bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/absolute/path/to/go2_map.pcd rviz:=true
```

然后：

1. Fixed Frame 设置为 `map`、`map_<name>` 或 `localization_world`，视当前模式而定。
2. 使用 `2D Pose Estimate` 给 `/initialpose`。
3. 观察 `/localization_3d_confidence` 是否稳定高于阈值。

## 7. rosbag 回放

rosbag 回放时才使用仿真时间：

```bash
roslaunch open3d_loc localization_3d_go2.launch map_path:=/absolute/path/to/go2_map.pcd use_sim_time:=true rviz:=true
rosbag play xxx.bag --clock
```

不要启动 `livox_ros_driver2 msg_MID360.launch`，除非 rosbag 里没有 `/livox/lidar` 和 `/livox/imu`。

## 8. Claude 修改后的验收清单

Claude 完成修改后至少检查：

```bash
git diff -- FAST_LIO/launch/mapping_mid360_go2.launch
git diff -- open3d_loc/launch/open3d_loc_go2.launch
git diff -- open3d_loc/launch/localization_3d_go2.launch
git diff -- FAST_LIO/config/mid360_go2.yaml
git diff -- open3d_loc/CMakeLists.txt
git diff -- open3d_loc/config/maps_go2_example.yaml
```

必须满足：

- GO2 实机默认 `use_sim_time=false`。
- rosbag 可以通过 `use_sim_time:=true` 恢复。
- GO2 默认使用 `map_path` 单地图。
- 多地图只有 `use_multimap:=true` 时启用。
- 默认不再加载 `vsis515` / `vsis516`。
- 默认不再 `pcd_save_en=true`。
- `Open3D_DIR` 不再无条件覆盖用户传入值。
- `msg_MID360.launch` 的 `xfer_format` 仍然是 `1`。
- G1 launch/config 没有被误改。

## 9. 一句话结论

当前算法可以跑真实 GO2，但当前仓库的 GO2 默认配置更像 rosbag/实验环境。上车前必须至少改掉 GO2 的 `use_sim_time=true`、实验室多地图默认加载、PCD 默认保存、Open3D 硬编码路径，并人工确认 MID360 网络 IP、安装外参、`base_link -> imu_link` TF 和现场点云地图。
