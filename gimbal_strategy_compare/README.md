# gimbal_strategy_compare — 对比策略

包含两个节点和一个离线工具，用于云台策略对比。

## 编译

```bash
cd ~/loc_ws
catkin_make --only-pkg-with-deps gimbal_strategy_compare
```

依赖: ROS noetic, PCL 1.8+, Eigen3, cyber_msgs, voxel_motion_strategy。

---

## 节点与工具

### 1. filter_plane_points — 离线平面点筛选

从全局点云地图中筛选属于"平面"的点（墙面、地面等），输出过滤后的点云。

```bash
# 基本用法
./devel/lib/gimbal_strategy_compare/filter_plane_points \
  input.ply output.ply [voxel_m=0.5] [radius_m=2.0] [curv_thresh=0.03]

# 示例
./devel/lib/gimbal_strategy_compare/filter_plane_points \
  /home/loc/loc_ws/data/dianwanghesai/map-01new.ply \
  /home/loc/loc_ws/data/dianwanghesai/plane_map.ply
```

**参数说明**:
- `voxel_m` (默认 0.5): 降采样分辨率 (m)，减少计算量
- `radius_m` (默认 2.0): 法向量估计的邻域搜索半径 (m)
- `curv_thresh` (默认 0.03): 曲率阈值，PCL curvature < 此值的点视为平面点。越小越严格

**原理**: 对每个点计算局部邻域的法向量和曲率。曲率 = λ₀/(λ₀+λ₁+λ₂)，低曲率表示局部表面平坦。

### 2. plane_strategy_node — 平面点计数策略

核心思路：在 LiDAR 坐标系下，找到 FoV 内包含平面点最多的朝向。

```bash
roslaunch gimbal_strategy_compare plane_strategy.launch
```

**订阅话题**:

| 话题 | 类型 | 用途 |
|------|------|------|
| `/iekf3d/odometry` | `nav_msgs::Odometry` | 车辆位姿 |
| `/pan` | `std_msgs::Float64MultiArray` | 云台 pan 反馈 (data[1], 度) |
| `/tilt` | `std_msgs::Float64MultiArray` | 云台 tilt 反馈 (data[1], 度) |

**发布话题**:

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gimbal_cmd` | `cyber_msgs::GimbalCommand` | 云台控制指令 (PAN=0x4B, TILT=0x4D) |

**参数**: 见 `configs/plane_strategy.yaml`。

**算法流程**:
```
预加载 plane_map.ply → 构建 kd-tree
  ↓
每 rotation_period (0.5s):
  ├─ 获取 odom/pan/tilt
  ├─ 每 strategy_period (2s) 重评估:
  │   1. kd-tree 半径搜索获取局部平面点
  │   2. 转换到 LiDAR 坐标系
  │   3. 球面投影分 bin (CountGrid)
  │   4. 积分图 (CountSAT)
  │   5. 矩形搜索 — 找平面点最多的 FoV 方向
  │   6. 更新目标 target
  └─ 向 target 步进 (运动约束) → 发 /gimbal_cmd
```

### 3. local_map_publisher — 局部点云发布

从全局地图中提取以 LiDAR 为中心的局部点云，供外部对比策略使用。

```bash
rosrun gimbal_strategy_compare local_map_publisher _sample_period:=2.0
```

**订阅话题**:

| 话题 | 类型 | 用途 |
|------|------|------|
| `/iekf3d/odometry` | `nav_msgs::Odometry` | 车辆位姿 |

**发布话题**:

| 话题 | 类型 | 坐标系 | 说明 |
|------|------|--------|------|
| `/local_map` | `sensor_msgs::PointCloud2` | `lidar_frame` | 局部点云 (x/y/z), 半径 150m, 0.5m 降采样 |

**参数**: 通过 ROS param 指定:
- `pcd_path`: 全局地图路径 (默认 `map-01new.ply`)
- `sample_period`: 采样周期 (默认 2.0s)
- `range`: 提取半径 (默认 150m)
- `lidar_extinct`: LiDAR 外参 `[tx,ty,tz,roll,pitch,yaw]`

---

## 生成对比用 rosbag

用于向外部合作者交付数据：

```bash
# 终端1: 播放原始 bag (使用 bag 内时间戳)
rosbag play --clock ~/loc_ws/data/exp/2026-06-27-17-43-46.bag

# 终端2: 启动局部点云发布
rosrun gimbal_strategy_compare local_map_publisher _sample_period:=2.0

# 终端3: 录制
./record.sh
```

输出 `output.bag` 包含的话题见 `bag-readme.md`。

---

## 坐标系

### `lidar_frame`
```
原点: LiDAR 传感器安装位置
X 轴: LiDAR 正前方
Y 轴: LiDAR 左侧
Z 轴: 垂直向上
```

### `map`
全局 ENU 坐标系，里程计 `/iekf3d/odometry` 使用的参考系。

### 坐标变换
```
map → vehicle_frame (odometry)
      └→ lidar_frame
      (LiDAR 安装外参: T=[-0.2, 0.25, 1.43], R≈I)
```
