# gimbal_strategy_compare — 对比策略数据生成

## 节点: local_map_publisher

从全局点云地图中按车辆位姿提取局部子集，转换到 LiDAR 坐标系发布。
接收方拿到的是和策略相同的环境信息，只是以点云（而非 ERP 图像）的形式表达。

### 数据来源

- `map-01new.ply` — IEKF 定位用点云地图，750万点，字段 x/y/z

### 订阅话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/iekf3d/odometry` | `nav_msgs::Odometry` | 车辆位姿，用于确定局部区域中心 |

### 发布话题

| 话题 | 类型 | 坐标系 | 说明 |
|------|------|--------|------|
| `/local_map` | `sensor_msgs::PointCloud2` | `lidar_frame` | 以 LiDAR 原点为中心的局部点云（x/y/z），范围 150m |

### 坐标系 `lidar_frame`

```
原点: LiDAR 传感器安装位置 (车辆坐标系下: [1.08, 0, 1.643])
X 轴: LiDAR 前方
Y 轴: LiDAR 左侧
Z 轴: 垂直向上
```

该坐标系与策略节点中 ERP 投影的坐标系完全一致。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_path` | map-01new.ply 路径 | 定位点云文件 |
| `sample_period` | 2.0 | 采样周期 (秒) |
| `range` | 150.0 | 提取半径 (米) |
| `lidar_extinct` | `[1.08,0,1.643,1.67,0.49,89.24]` | LiDAR 外参 |

### 生成 rosbag 流程

```bash
# 终端1: 播放原始 bag (使用 bag 内时间戳)
rosbag play --clock ~/loc_ws/data/exp/2026-06-27-17-43-46.bag

# 终端2: 启动本地图发布节点
rosrun gimbal_strategy_compare local_map_publisher _sample_period:=2.0

# 终端3: 录制 (原始数据 + local_map)
rosbag record -O output.bag \
  /iekf3d/odometry \
  /gimbal_cmd \
  /pan /tilt \
  /local_map
```

### 交付给原作者的话题

| 话题 | 坐标系 | 含义 |
|------|--------|------|
| `/local_map` | `lidar_frame` | 每个采样位姿下的局部点云，接收方唯一输入 |
| `/iekf3d/odometry` | `map` | 车辆位姿参考 |
| `/gimbal_cmd` | — | 当前策略输出的云台指令（对比用） |
| `/pan` `/tilt` | — | 云台反馈角度（对比用） |

### 局部点云的转换过程

```
1. 根据 /iekf3d/odometry 得到车辆在 map 坐标系下的位姿
2. 通过 LiDAR 外参计算 LiDAR 传感器在 map 坐标系下的位置
3. 在全局点云中查询以 LiDAR 为中心、range 为半径的所有点
4. 将选中的点从 map 坐标系转换到 lidar_frame
5. 发布
```

### 原作者需要实现的函数签名（建议）

```
输入: sensor_msgs::PointCloud2 (局部点云, lidar_frame, x/y/z/intensity)
输出: (pan_deg, tilt_deg)  — 云台目标角度 (pelco 0-360° 格式)
```
