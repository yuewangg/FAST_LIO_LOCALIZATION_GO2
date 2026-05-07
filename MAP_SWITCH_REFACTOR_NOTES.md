# Open3D Loc Multi-Map Refactor Notes

本文档给 opencode 使用。目标不是继续调阈值，而是修正当前多地图切换的坐标系语义，并修复运行中发送 `/initialpose` 后地图/定位跳变的问题。

## 背景

当前运行目标：

- 启动：`roslaunch open3d_loc localization_3d_go2.launch`
- 回放：`rosbag play /root/loc_ws/data/vsis_2026-05-06-17-39-20.bag --clock`
- 启动地图：`vsis515`
- 进入切换区域后静默验证 `vsis516`
- 切换区域大约在 bag elapsed `89s` 附近触发

当前代码位置：

- `open3d_loc/src/global_localization.cpp`
- `open3d_loc/launch/open3d_loc_go2.launch`
- `open3d_loc/config/loc_param_go2.yaml`

当前实现里所有发布结果基本都固定使用 frame `"map"`：

- `/map` 点云：`header.frame_id = "map"`
- `/odom2map`：`header.frame_id = "map"`，child 为 `"odom"`
- TF：`map -> odom`
- `/baselink2map`、`/motionlink2map`、`/localization_3d`：`header.frame_id = "map"`

这隐含了一个强假设：所有地图都已经在同一个全局 `map` 坐标系下对齐。现在用户明确希望 **地图之间是分离坐标系**，因此这个假设必须移除。

## 已观察到的问题

### 1. 多地图切换仍按“同一 map 坐标系”处理

当前 `SwitchActiveMap()` 只是替换 active map 指针，并继续发布同一个 frame `"map"`。如果 `vsis515` 和 `vsis516` 是两张独立局部地图，那么切换后：

- frame 名字还是 `"map"`，但 frame 的实际坐标含义已经变了。
- RViz/下游节点会把两个不同局部坐标系误认为同一个坐标系。
- 这会表现为定位结果错误、地图/机器人跳变，或者看似 fitness 达标但空间位置不可信。

之前验证中出现过两类现象：

- 使用大范围 multiscale ICP 时，`vsis516` fitness 可以超过 `0.8`，但 `odom2map` 会跳到约 `(-3.9, -1.1, yaw 22deg)` 一类的错误局部最优。
- 加入局部 ICP 和位姿修正限制后，错误大跳变会被拒绝，但切换后主定位置信度仍有一段降到 `0.47~0.65`，说明单纯阈值补丁不足以解决“分离地图坐标系”的架构问题。

### 2. `next_initialpose` / `initialpose` 语义不清

当前代码把 `initialpose` 或 `next_initialpose` 直接作为 `mat_odom2map_` 使用。实际更合理的语义应该是：

- 人配置的 `initialpose`：机器人 `base_link` 在某张地图局部坐标系下的初始位姿，即 `T_map_base`。
- 切换配置的 `next_initialpose`：进入切换区时，机器人在下一张地图局部坐标系下的预期位姿，即 `T_next_map_base`。

然后运行时根据当前 FAST-LIO odom 计算：

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

这比把配置直接当作 `T_map_odom` 更正确，因为 `odom` 是运行时坐标系，不应该要求离线配置固定它。

### 3. `/initialpose` 回调会导致跳变

当前 `CallbackInitialPose()` 的核心逻辑是：

```text
mat_initialpose_ = pose from /initialpose
mat_odom2map_ = mat_initialpose_
```

这是错误的。ROS/RViz 的 `/initialpose` 通常表示机器人在 map 下的位姿 `T_map_base`，不是 `T_map_odom`。正确做法是拿当前 `T_odom_base` 反算：

```text
T_map_odom = T_map_base_from_initialpose * inverse(T_odom_base_current)
```

当前实现会把 base pose 当成 odom pose，车辆运行一段时间后 `T_odom_base` 已经不为 identity，所以手动给初值会引入等同于 odom 偏移的跳变。

另外还有几个连带问题：

- 回调里硬编码 `loc_initialized_ && loc_fitness_ > 0.99` 时忽略初值，不使用 `confidence_loc_th_`，用户主动给初值时不应被静默忽略。
- 设置 `mat_odom2map_` 后没有系统性重置 Kalman filter、`last_loc_`、当前 submap。
- 多个线程读写 `mat_baselink2odom_`、`mat_odom2map_`、`mat_baselink2map_` 的锁不一致，切换或 initialpose 时可能和 odom 回调/定位线程竞态。

## 目标语义

### 地图是独立局部坐标系

每张地图应该有自己的 frame，例如：

```text
map_vsis515
map_vsis516
```

内部状态应该明确表示：

```text
active_map_idx
active_map_frame
T_active_map_odom
T_active_map_base = T_active_map_odom * T_odom_base
```

切换地图不是“在同一个 map frame 里替换点云”，而是：

```text
active frame: map_vsis515  ->  map_vsis516
active transform: T_515_odom -> T_516_odom
```

如果下游需要一个视觉上连续的固定 frame，可以额外引入 display/world frame，但不要把它和地图局部 frame 混在一起。

建议 frame 设计：

```text
localization_world
  └── map_vsis515
      └── odom
          └── base_link

切换后：

localization_world
  └── map_vsis516
      └── odom
          └── base_link
```

`localization_world -> map_<active>` 可以在切换时计算，用于 RViz 保持显示连续；如果不需要连续显示，也可以只发布 `map_<active> -> odom`，但必须让消息 frame_id 反映 active map。

### 发布规则

最低要求：

- `/map.header.frame_id = active_map_frame`
- `/localization_3d.header.frame_id = active_map_frame`
- `/baselink2map.header.frame_id = active_map_frame`
- `/motionlink2map.header.frame_id = active_map_frame`
- `/odom2map.header.frame_id = active_map_frame`
- TF 不再固定发布 `"map" -> "odom"`，而是发布 `active_map_frame -> odom`

兼容建议：

- 保留 `/localization_3d_active_map`
- 新增或扩展 `/localization_3d_status`，明确输出 `VERIFYING`、`SWITCHED`、`MANUAL_INITIALPOSE`、`REJECTED`
- 如果必须保留 legacy `"map"`，应作为 display frame 使用，不能再代表所有地图的局部 frame。

## 建议重构方案

### 1. 扩展 MapEntry

建议字段：

```text
name
path
frame_id            # 默认 "map_" + name
map_coarse
map_fine
bbox_min/bbox_max
switch_zone         # 当前 active map frame 下的区域
initialpose         # T_this_map_base，单位 [x,y,z,roll,pitch,yaw_deg]
next_initialpose    # T_next_map_base，单位 [x,y,z,roll,pitch,yaw_deg]
```

注意：`initialpose` 和 `next_initialpose` 都应解释为 `map -> base_link`，不要直接解释为 `map -> odom`。

### 2. 统一 transform 命名

当前变量名 `mat_odom2map_` 实际表示 `T_map_odom`。建议至少在注释里修正，最好重命名：

```text
mat_map2odom_       # 如果使用 T_map_odom 这个数学方向，则命名应清楚
```

如果不重命名，也要在文档/注释中明确：

```text
mat_odom2map_ means T_map_odom: transforms points from odom frame into active map frame.
```

### 3. 初始定位逻辑

启动时不要直接：

```text
mat_odom2map_ = mat_initialpose_
```

而是等待第一帧 odom 后：

```text
T_active_map_base_guess = active_map.initialpose or launch initialpose
T_active_map_odom_guess = T_active_map_base_guess * inverse(T_odom_base_current)
mat_odom2map_ = T_active_map_odom_guess
LocalizationInitialize()
```

### 4. 静默切换逻辑

进入当前 active map 的 `switch_zone` 后：

```text
T_odom_base = current fast-lio odom
T_next_map_base_guess = active_map.next_initialpose
T_next_map_odom_guess = T_next_map_base_guess * inverse(T_odom_base)
```

然后对 candidate map 做 shadow localization：

- 使用 candidate map 的局部 frame。
- source 仍在 odom frame 中裁剪。
- target 在 candidate map frame 中按 `T_next_map_odom_guess * T_odom_base` 附近裁剪。
- ICP 初值是 `T_next_map_odom_guess`。
- ICP 只允许小幅修正，避免重复结构导致高 fitness 错误匹配。

推荐验证条件：

```text
fitness >= verify_fitness
连续 N 次成功
ICP 修正量 <= verify_max_translation / verify_max_yaw_deg
候选位姿不能明显违反 odom 连续性
```

提交切换时：

```text
active_map_idx = candidate
active_map_frame = candidate.frame_id
mat_odom2map_ = best_verified_T_candidate_map_odom
mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_
reset filters
force_resubmap = true
last_loc = invalid
publish active map pointcloud with candidate.frame_id
publish status SWITCHED
```

建议保存“验证窗口内最优 candidate pose”，不要简单使用第 N 次达标那一帧。

### 5. 修复 `/initialpose`

`CallbackInitialPose()` 应解释为用户给出的 active map 下机器人位姿。

核心修复：

```text
T_active_map_base = pose from /initialpose
T_odom_base = current mat_baselink2odom_
T_active_map_odom = T_active_map_base * inverse(T_odom_base)

mat_odom2map_ = T_active_map_odom
mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_
```

如果 `/initialpose.header.frame_id` 不是当前 active map frame，而是 display/world frame，需要先通过 TF 转到 active map frame，再计算 `T_active_map_odom`。

回调后必须：

```text
reset kf_baselink_x/y/z
reset kalman_filter_odom2map if enabled
force_resubmap = true
last_loc = invalid
publish status MANUAL_INITIALPOSE
```

不要在 `/initialpose` 回调里重新发布或移动地图点云。用户给初值应该改变机器人定位初值，不应该改变当前地图坐标系。

是否重新初始化：

- 推荐设置一个 `manual_reinit_pending_` 标志，让定位线程在自己的循环里执行 `LocalizationInitialize()`，避免在 ROS 回调线程里阻塞。
- 用户显式给 `/initialpose` 时不要因为当前 `loc_fitness_ > 0.99` 而忽略。可以只在配置项允许时过滤，但默认应接受。

### 6. 线程安全

目前 `CallbackBaselink2Odom()`、`Localization()`、`MapSwitchMonitor()`、`CallbackInitialPose()` 都会读写 transform 状态。需要统一锁保护这些变量：

```text
mat_baselink2odom_
mat_odom2map_
mat_odom2map_kalman_
mat_baselink2map_
loc_initialized_
loc_fitness_ if used across threads
```

不要只在部分路径使用 `lock_mat_odom2map_`。建议重命名为 `lock_state_` 或拆出 `lock_tf_state_`。

## 对 opencode 的修改要求

请按以下优先级修改：

1. 修复 `/initialpose` 的数学关系，不再把 `T_map_base` 直接赋给 `T_map_odom`。
2. 引入每张地图独立 `frame_id`，发布消息和 TF 使用 active map frame。
3. 将 `initialpose` / `next_initialpose` 统一解释为 `T_map_base`，运行时转换为 `T_map_odom`。
4. 调整静默切换：candidate map 验证在 candidate map 局部 frame 内完成，提交时切换 active frame 和 `T_candidate_map_odom`。
5. 重置滤波器、submap、`last_loc_`，避免切换或 initialpose 后沿用旧地图状态。
6. 梳理 transform 状态锁，避免 odom 回调、定位线程、切换线程、initialpose 回调竞态。

不要做无关重构，不要改 FAST_LIO 和 livox_ros_driver2。

## 验收方法

### 构建

```bash
source /root/loc_ws/devel/setup.bash
catkin_make --pkg open3d_loc
```

### Bag 验证

终端 1：

```bash
source /root/loc_ws/devel/setup.bash
roslaunch open3d_loc localization_3d_go2.launch
```

终端 2：

```bash
source /root/loc_ws/devel/setup.bash
rosbag play /root/loc_ws/data/vsis_2026-05-06-17-39-20.bag --clock
```

终端 3：

```bash
rostopic echo /localization_3d_status
rostopic echo /localization_3d_active_map
rostopic echo /localization_3d/header
rostopic echo /map/header
```

预期：

- 启动 active map 为 `vsis515`。
- `/map.header.frame_id` 初始为 `map_vsis515` 或配置的 515 frame。
- bag elapsed 约 `89s` 后开始 `VERIFYING:vsis516`，但不立即切换。
- 连续满足验证条件后输出 `SWITCHED:vsis515->vsis516`。
- 切换后 `/map.header.frame_id` 变为 `map_vsis516` 或配置的 516 frame。
- `/localization_3d.header.frame_id` 与 active map frame 一致。
- 不应再出现用同一个 `"map"` frame 表示两张分离地图的情况。

### `/initialpose` 验证

系统运行中用 RViz 或命令发布 `/initialpose`：

- 地图点云不应因为 initialpose 被重新解释而跳变。
- active map 不应被改变。
- 机器人定位应围绕用户给定初值重新收敛。
- `/localization_3d_status` 应输出类似 `MANUAL_INITIALPOSE`。
- 后续定位线程应重建 submap，不能沿用旧 submap。

## 关键判断

如果地图 `vsis515` 和 `vsis516` 没有离线对齐到同一全局坐标系，那么任何继续固定使用 `"map"` frame 的方案都只是调参补丁，不能从根上正确。正确方向是把 active map frame 作为定位结果的一部分，让每张地图拥有独立局部坐标系。

## 第二轮评审：opencode 修改后仍然切换错误

用户反馈：opencode 已经按上一版文档修改后，地图切换后依然错误；在 RViz 下观察，从切换到第二张地图的第一帧开始，当前点云、新加载地图、定位结果就是错的。

这说明现在的问题不再只是 frame 名称是否分离，而是 **切换瞬间使用的候选地图位姿本身不可靠**，并且主定位线程在切换边界可能仍有旧地图状态污染。

### 当前代码中的高风险点

当前 `open3d_loc/launch/open3d_loc_go2.launch` 仍然配置：

```yaml
maps:
  - { name: vsis515, ..., next_initialpose: [0, 0, 0, 0, 0, 0] }
  - { name: vsis516, ..., initialpose: [0, 0, 0, 0, 0, 0] }
```

opencode 已将 `next_initialpose` 解释为 `T_next_map_base`，这是正确方向；但对于两张分离地图，`[0,0,0,0,0,0]` 表示“切换时机器人在 516 地图原点且 yaw=0”。这通常不成立。只要这个入口位姿不对，后面的 shadow ICP 就是在错误初值附近做局部验证，可能会：

- 直接裁剪到 516 地图错误区域；
- 在重复结构中得到看似可接受的 fitness；
- 切换第一帧就把 `/cloud_registered_1` 投到 516 的错误位置；
- RViz 中表现为新地图、当前点云、定位结果三者从第一帧就不一致。

另一个高风险点是 `localization_world -> map_<active>` 当前被固定发布为 identity：

```text
localization_world -> map_vsis515 = Identity
localization_world -> map_vsis516 = Identity
```

如果 RViz Fixed Frame 是 `localization_world`，这等价于把两张独立地图的原点强行叠到一起。对于“地图分离”的需求，这只能作为最简单显示方式，不能表达 515 与 516 在显示世界里的连续关系。如果期望 RViz 里切换瞬间视觉连续，需要计算并发布非 identity 的 `T_world_map516`。

还有一个实现层面的边界问题：主定位线程持有本地变量 `map_fine_crop`，它可能是旧 active map 裁出来的 submap。切换时虽然设置了 `force_resubmap_`，但已经在途的一轮 localization 仍可能用旧 target、旧 active 状态或旧 `reg_matrix` 继续跑完并写回 `mat_odom2map_`。即使不是每次都发生，这也足以造成“切换第一帧错误”。

### 需要先澄清的语义

对分离地图，不能再把“自动切换”理解为在 515 坐标下走到某个区域后，靠 ICP 自动知道 516 坐标在哪里。系统必须拥有以下两类信息之一：

1. **显式入口位姿**：进入切换区域时，机器人在 516 地图坐标系中的期望位姿 `T_516_base_at_entry`。
2. **地图间连接关系**：`T_516_515` 或 `T_world_map515 / T_world_map516`，可把当前 515 位姿转换到 516。

如果两者都没有，只靠当前一帧点云和局部 ICP，无法保证在重复走廊/房间结构里切到正确位置。fitness 不是充分条件。

### 必须新增：入口位姿标定流程

opencode 需要把 `next_initialpose` 从“随便填 0”改成可标定、可验证的参数。

建议实现一个标定/记录流程：

1. 正常用 `vsis515` 跑 bag，到切换区域附近暂停或让系统进入 `VERIFYING`。
2. 临时加载 `vsis516` 为候选地图，但不要提交切换。
3. 用户在 RViz 中针对 `map_vsis516` 给 `/initialpose`，或调用一个服务设置 candidate pose。
4. 系统把该 pose 记录为：

```text
vsis515.next_initialpose = T_map_vsis516_base_at_switch
```

5. 将该值输出到日志和 `/localization_3d_status`，格式可直接复制回 launch/yaml：

```yaml
next_initialpose: [x, y, z, roll, pitch, yaw]
```

验收标准：如果把记录出的 `next_initialpose` 写回配置，再跑同一个 bag，进入 516 后第一帧点云应该已经落在 516 正确区域附近，ICP 只做小修正。

如果不想做交互标定，也可以离线从 bag 中抽取切换区域的 `/cloud_registered_1` 子图，对 `vsis516.pcd` 做一次全局配准/人工确认，然后记录 `T_516_base_at_entry`。但最终仍必须落成显式参数，不能继续使用全零入口位姿。

### 必须新增：切换前后的严格一致性检查

当前 `VERIFYING` 只看 candidate fitness 和相对初值修正量。对分离地图还需要检查“切过去后的第一帧是否真的能被主定位接受”。

建议切换流程改为两阶段提交：

```text
VERIFYING_CANDIDATE
  - 使用 next_initialpose 得到 T_candidate_map_odom_guess
  - 在 candidate map 中 shadow ICP
  - 保存最近 N 次的 candidate result
  - 选择窗口内 fitness 最高且 delta 最小的 best_candidate

PRECOMMIT
  - 用 best_candidate 裁 candidate submap
  - 用当前 scan 按主定位同样的 RegistrationIcp/EvaluateRegistration 跑一次
  - 要求 post_switch_fitness >= post_switch_min_fitness
  - 要求 T_candidate_map_base 与 next_initialpose 不超过 max jump

COMMIT
  - 原子切换 active map、active frame、T_candidate_map_odom、submap cache
  - 发布新 /map
  - 第一个发布出去的 /localization_3d 必须来自 candidate map 验证结果

ROLLBACK/REJECT
  - 如果 PRECOMMIT 失败，不切图，只发布 REJECTED
```

关键点：不要让“刚好第 N 次达标”的那一帧直接作为切换位姿。应保存验证窗口内的 best pose，并用主定位同样路径做一次 precommit 验证。

### 必须修复：切换瞬间的旧 submap / 旧定位结果污染

opencode 需要为定位线程增加 map epoch/generation。

建议字段：

```text
std::atomic<uint64_t> map_epoch_{0};
```

切换时：

```text
map_epoch_++
active_map_idx = candidate
active_map_frame = candidate.frame_id
mat_odom2map = T_candidate_map_odom
clear/rebuild current target submap
force_resubmap = true
last_loc = invalid
```

定位线程每轮开始时 snapshot：

```text
epoch_begin = map_epoch_
active_idx_begin = active_map_idx
frame_begin = active_map_frame
T_map_odom_begin = mat_odom2map
```

本轮结束写回前必须检查：

```text
if (epoch_begin != map_epoch_) {
    discard this registration result;
    continue;
}
if (active_idx_begin != active_map_idx) {
    discard this registration result;
    continue;
}
```

这样可以避免切换期间仍在运行的一轮旧地图 ICP 把旧结果写回新地图状态。

同时，`map_fine_crop` 不能跨地图复用。建议把 cached submap 和它所属的 `active_idx/epoch` 绑定：

```text
cached_submap_epoch
cached_submap_map_idx
```

只要 map idx 或 epoch 改变，必须立即丢弃旧 submap 并用新 active map 重裁。

### 必须修复：RViz 的 display/world 语义

如果 RViz Fixed Frame 使用 `localization_world`，那么 `localization_world -> map_vsis516` 不能无脑 identity。否则两张分离地图会在 RViz 中共享原点，切换显示必然容易误判。

有两种可选方案，opencode 需要明确选一种。

方案 A：不追求跨地图视觉连续，RViz 固定到 active map frame

- `/map.header.frame_id = map_vsis515` 或 `map_vsis516`
- `/localization_3d.header.frame_id = active map frame`
- RViz fixed frame 跟随 active map，或用户手动切换
- 不发布 `localization_world -> map_*`，避免误导

方案 B：追求 RViz 视觉连续，维护 display world

需要维护：

```text
T_world_active_map
```

启动 515 时可设：

```text
T_world_map515 = Identity
```

切到 516 前，取切换前在 world 下的 base pose：

```text
T_world_base_before = T_world_map515 * T_map515_base_before
```

candidate 验证得到：

```text
T_map516_base_verified
```

为了让 RViz 中机器人位置连续，计算：

```text
T_world_map516 = T_world_base_before * inverse(T_map516_base_verified)
```

然后发布：

```text
localization_world -> map_vsis516 = T_world_map516
map_vsis516 -> odom = T_map516_odom
```

这样新地图会在 `localization_world` 下移动到正确显示位置，当前点云、机器人和新地图才可能在 RViz 里连续。不要继续把所有 `localization_world -> map_*` 都发 identity。

注意：方案 B 只解决显示连续；定位算法内部仍应完全使用 active map 局部坐标。

### 必须补充日志和诊断输出

为了避免继续靠肉眼猜，切换前后必须输出以下量：

```text
active_map old/new
active_frame old/new
T_old_map_base_before
T_candidate_map_base_guess
T_candidate_map_base_verified
T_candidate_map_odom_committed
T_world_map_new if using localization_world
candidate fitness
post-switch precommit fitness
delta from next_initialpose
map_epoch before/after
cached submap map_idx/epoch
```

状态 topic 建议输出：

```text
VERIFYING:vsis516 fit=... guess_base=(...) verified_base=(...) dxy=... dyaw=...
PRECOMMIT:vsis516 fit=...
SWITCHED:vsis515->vsis516 epoch=...
REJECTED:vsis516 reason=bad_precommit
REJECTED:vsis516 reason=bad_initialpose_or_missing_entry_pose
```

### 对当前配置的直接判断

当前配置中的：

```yaml
next_initialpose: [0, 0, 0, 0, 0, 0]
```

不能作为分离地图自动切换的最终参数。它最多只能用于临时测试“516 地图原点附近是否恰好是切换入口”。从用户反馈“切到第二张地图第一帧就是错的”看，这个假设不成立。

opencode 下一步不应继续调 `verify_fitness`，而应：

1. 增加入口位姿标定/记录能力；
2. 用真实 `T_516_base_at_entry` 替换全零 `next_initialpose`；
3. 增加 precommit 验证和 map epoch；
4. 修正 `localization_world -> map_*` 的显示逻辑，或取消该 world frame 的 identity 伪连续显示。

## 第二轮修改任务清单

请 opencode 按这个顺序修改：

1. 添加 `map_epoch_`，让定位线程丢弃跨地图切换期间的旧 ICP 结果。
2. 将 cached submap 与 `map_idx/epoch` 绑定，切换后强制清空旧 submap，第一帧必须用新地图重裁。
3. 为 `next_initialpose` 增加“缺失/未标定”判断：如果是默认全零且未显式声明有效，不允许自动切图，只进入 `VERIFYING` 或 `NEED_CALIBRATION`。
4. 增加候选入口位姿记录功能，输出可复制回 launch/yaml 的 `next_initialpose`。
5. 切换前增加 `PRECOMMIT`：用 candidate map、best candidate pose、主定位同样配准路径验证一次；失败则不切。
6. 修正 `localization_world -> map_*`：要么按方案 B 计算 `T_world_map_new`，要么去掉 identity world frame，避免 RViz 误导。
7. 在 `/localization_3d_status` 和 ROS log 中打印上面列出的关键矩阵/位姿/fitness/epoch。

第二轮验收标准：

- 没有真实 `next_initialpose` 时，系统不应自动提交切换，应提示 `NEED_CALIBRATION` 或 `REJECTED:bad_initialpose_or_missing_entry_pose`。
- 写入真实 `next_initialpose` 后，bag elapsed 约 `89s` 进入验证，只有 precommit 通过才切换。
- 切换提交后的第一帧 `/map`、`/cloud_registered_1`、`/localization_3d` 在 active map frame 下已经一致，不允许先错一帧再恢复。
- RViz 使用 `localization_world` 时，若选择方案 B，应视觉连续；若选择方案 A，不应发布会误导的 identity `localization_world -> map_*`。
