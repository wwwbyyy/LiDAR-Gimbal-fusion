# output.bag

## 话题

### `/local_map` — 局部点云地图 (sensor_msgs::PointCloud2)

从全局点云地图 ( 0.5m 降采样) 中提取的局部子集。

- **frame_id**: `lidar_frame`
- **字段**: `x`, `y`, `z` (float32, 米)
- **内容**: 以 LiDAR 传感器为中心、半径 150m 内的点云
- **发布频率**: 每 2s 一帧 (可调)

### `/iekf3d/odometry` — 车辆里程计 (nav_msgs::Odometry)

- **frame_id**: `iekf_map`
- **child_frame_id**: `vehicle_frame`

### `/gimbal_cmd` — 云台指令 (cyber_msgs::GimbalCommand)

策略节点发出的原始云台控制指令，类型定义：
```
std_msgs/Header header
uint8 cmd
float64 data
```

### `/pan`, `/tilt` — 云台角度反馈 (std_msgs::Float64MultiArray)

实验时的角度反馈值。
- `data[0]`: 时间戳 (double)
- `data[1]`: 角度值 (double, 度)

### `/iekf3d/global_map`, `/iekf3d/registrationd_cloud` - 可视化点云

- **frame_id**: `iekf_map`

在rviz里可视化实验时候的定位过程。

## 坐标系

### `lidar_frame` (局部点云坐标系)

```
原点:   LiDAR 传感器安装位置
X 轴:   LiDAR 正前方
Y 轴:   LiDAR 左侧
Z 轴:   垂直向上
```

这是云台在 pan=0°, tilt=0° 时 LiDAR 的朝向。

### `map` (全局参考坐标系)

里程计 `/iekf3d/odometry` 使用的全局 ENU 坐标系。

### 坐标系关系

```
map → vehicle_frame (odometry)
      └→ lidar_frame  
      (LiDAR 安装外参: T=[-0.2, 0.25, 1.43], R≈I)
```
