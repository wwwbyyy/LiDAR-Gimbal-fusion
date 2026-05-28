# Voxel Motion Strategy — 设计分析与实现 Pipeline

## 一、方法

### A. 矩阵 S、λ_min 与 score

S = Σ nᵀn 是矩形区域内法向量的二阶矩矩阵。λ_min 反映法向量方向多样性：

- 单一平面 → S 秩为 1，λ_min ≈ 0
- 墙角（正交平面）→ S 秩为 2，λ_min 中等
- 复杂三面角 → S 秩为 3，λ_min 最大

λ_min 越大 → 几何结构越丰富 → ICP 约束越完整。引入像素覆盖 N_effective：

```
score = λ_min - a·pitch
```

- `λ_min`：S = Σ nᵢnᵢᵀ 的最小特征值。tr(S) = Σ|nᵢ|² = N_eff，所以 λ_min 已隐含覆盖密度，不需二次加权
- `-a·pitch`：pitch 越小（越向上看）分越高。a > 0

### B. Yaw 运动约束（替代 δyaw 惩罚）

不使用 score 中的 δyaw 惩罚项。改用云台角速度/角加速度约束来剪枝搜索空间：

```
给定: current_yaw, current_vel (角速度), dt (策略周期),
      ω_max (最大角速度), α_max (最大角加速度)

可行角速度范围: vel' ∈ [current_vel - α_max·dt, current_vel + α_max·dt]
                ∩ [-ω_max, ω_max]

可行 yaw 范围: yaw' ∈ [current_yaw + vel'_min·dt, current_yaw + vel'_max·dt]
               再叠加 ±½·α_max·dt² 的加速度效应
```

带死区 δ_deadzone（不锁定在附近）：

```
yaw_min = current_yaw + min(0, current_vel - α_max·dt)·dt - ½·α_max·dt² - δ_deadzone
yaw_max = current_yaw + max(0, current_vel + α_max·dt)·dt + ½·α_max·dt² + δ_deadzone
clamp to: [current_yaw - ω_max·dt, current_yaw + ω_max·dt]
```

**效果**：
- 候选 yaw 搜索范围从全局 360° 缩小到运动可达范围（通常 ±20-40°）
- ERP 投影时只处理该水平区间（+ FoV margin），剪枝大量 ray-cast
- 不需要调 b 参数和 f(δyaw) 形式，参数更具物理意义

### C. 体素法向量一致性筛选 — 暂时跳过

经过测试，单独使用 R（法向量半球一致性）在 1m 和 0.5m 分辨率下均无法有效区分建筑立面和树叶。体素构建时暂不对法向量做筛选——所有被占据的体素直接存储其平均法向量。后续可在 ERP 投影阶段通过 S 矩阵的自然惩罚来处理不稳定区域。

### D. Octomap vs Dense Grid

采用 Octomap 存储体素地图：
- 户外稀疏场景下 ray-cast 通过空洞跳跃大幅减少步数
- 150m 范围对 Dense 不可行，对 Octomap 无压力

### E. 积分图的水平扩展

ERP 水平 360°，Avia FoV 矩形在 yaw 接近边界时会跨越 0°/360°。构建积分图时在右侧扩展 FoV_width 列。

### F. 暂不考虑

- 时间一致性 / 前瞻位姿（后续迭代）
- IEKF 协方差方向加权（后续迭代）

---

## 二、数据流与 Pipeline

```
┌──────────────────────────────────────────────────────────────────┐
│                  OFFLINE: 体素地图构建                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  全局地图 (.ply, 含法向量)                                        │
│       │                                                          │
│       ▼                                                          │
│  OctomapBuilder                                                   │
│  ├── 加载点云 + 法向量                                           │
│  ├── 构建 Octomap (resolution: ~1m)                               │
│  ├── 每个节点: 存储点位 + 平均法向量 (不做一致性筛选)              │
│  └── 输出: VoxelOctomap (序列化存储)                              │
│                                                                  │
├──────────────────────────────────────────────────────────────────┤
│                  ONLINE: 策略更新循环 (2-5 Hz)                    │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ① getCurrentState()                                              │
│     ├── 订阅 /localization/estimation → 位姿                      │
│     └── 订阅 /pan 反馈 → current_yaw, current_yaw_vel            │
│                                                                  │
│  ② computeFeasibleYawRange(current_yaw, current_vel, dt, params) │
│     ├── 角速度 + 角加速度约束                                     │
│     ├── 死区扩展                                                   │
│     └── 输出: [yaw_min, yaw_max]                                  │
│                                                                  │
│  ③ ERPProjector::project(octomap, pose, yaw_range, params)       │
│     ├── 以位姿为球心                                              │
│     ├── 水平: 仅 yaw_range + FoV margin (剪枝!)                   │
│     ├── 垂直: ~140° (覆盖 pitch_candidate)                        │
│     ├── 范围: 半径 150m                                           │
│     ├── 分辨率: ~1°/pixel                                         │
│     ├── Octree ray-cast → 第一个命中体素 → 法向量 n               │
│     └── 输出: ERPDepthImage (水平范围缩小)                         │
│                                                                  │
│  ④ IntegralImage::build(erp_image)                               │
│     ├── 每个 pixel: M = n * n^T (3x3 对称, 6 分量)               │
│     ├── 每个 pixel: occupied (0/1)                                │
│     └── 7 个标量积分图 (6 矩阵分量 + 1 占用计数)                   │
│                                                                  │
│  ⑤ RectangleSearch::search(integral_imgs, yaw_range, pitch_range)│
│     ├── Avia FoV 矩形: w ≈ 70.4°/res, h ≈ 77.2°/res              │
│     ├── 候选 yaw: yaw_range 内, step 2-5°                        │
│     ├── 候选 pitch: [-50°, 20°], step 2-5°                       │
│     ├── 每个候选 O(1):                                            │
│     │   a) 积分图查 S (6 分量)                                    │
│     │   b) 积分图查 N_effective                                   │
│     │   c) λ_min ← SelfAdjointEigenSolver<3>                     │
│     │   d) score = λ_min - a·pitch_c               │
│     └── argmax → best_yaw, best_pitch                            │
│                                                                  │
│  ⑥ publishGimbalCmd(best_yaw, best_pitch)                        │
│     └── cyber_msgs::GimbalCommand → /gimbal_cmd                  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 三、目录/文件结构

```
voxel_motion_strategy/
├── include/voxel_motion_strategy/
│   ├── octomap_builder.h      # Octomap 构建 + 法向量均值
│   ├── erp_projector.h        # ERP 投影 + octree ray-cast (支持水平剪枝)
│   ├── integral_image.h       # 矩阵积分图
│   ├── rectangle_search.h     # 候选搜索 + score
│   └── yaw_constraint.h       # 运动约束 → 可行 yaw 范围
├── src/
│   ├── voxel_motion_strategy_node.cpp
│   ├── octomap_builder.cpp
│   ├── erp_projector.cpp
│   ├── integral_image.cpp
│   ├── rectangle_search.cpp
│   └── yaw_constraint.cpp
├── launch/
│   └── voxel_strategy.launch
├── configs/
│   └── voxel_strategy.yaml
├── CMakeLists.txt
└── package.xml
```

---

## 四、关键参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `octomap_resolution` | 1.0 m | 体素分辨率 |
| `erp_resolution` | 1.0° | ERP 像素/度 |
| `erp_range_max` | 150 m | 射线最大距离 |
| `erp_vfov_min` | -90° | ERP 垂直下界 |
| `erp_vfov_max` | 50° | ERP 垂直上界 |
| `pitch_candidate_min` | -50° | 候选 pitch 下界 |
| `pitch_candidate_max` | 20° | 候选 pitch 上界 |
| `yaw_step` | 3° | 候选 yaw 搜索步长 |
| `pitch_step` | 3° | 候选 pitch 搜索步长 |
| `fov_horizontal` | 60° | Avia 圆形 FoV 内接矩形 (水平) |
| `fov_vertical` | 68° | Avia 圆形 FoV 内接矩形 (垂直) |
| `weight_pitch` (a) | 待调 | pitch 惩罚权重 |
| `max_angular_velocity` | 待定 | 云台最大角速度 (deg/s) |
| `max_angular_acceleration` | 待定 | 云台最大角加速度 (deg/s²) |
| `dyaw_deadzone` | 待定 | 死区角度，避免锁在附近 |
| `strategy_update_rate` | 4 Hz | 策略更新频率 (确定 dt) |

---

## 五、实施计划

### Step 0: octomap_test 验证 — 已完成
- 已测试 1m 和 0.5m 体素下 R + planarity 的区分能力
- 结论: 体素法向量一致性筛选跳过，不做

### Step 1: 核心模块实现（按依赖顺序）

1. `octomap_builder` — Octomap 构建 + 平均法向量 + 序列化
2. `yaw_constraint` — 角速度/角加速度约束 → 可行 yaw 范围
3. `erp_projector` — Octree ray-cast + 水平剪枝 ERP
4. `integral_image` — 7 个标量积分图
5. `rectangle_search` — 候选搜索 (yaw 受约束) + score = λ_min × N_eff - a·pitch

### Step 2: ROS 节点集成

- 订阅 `/localization/estimation` 和 `/pan`（获取当前 yaw/角速度）
- 发布 `/gimbal_cmd`
- YAML 参数加载

### Step 3: 调参验证

- 确定 `max_angular_velocity`、`max_angular_acceleration`（查阅云台规格）
- 确定 `weight_pitch` (a)
- rosbag 对比测试
