# Voxel Motion Strategy — 设计分析与实现 Pipeline

## 一、方法辨析

### A. 矩阵 S、λ_min 与 N_effective

S = Σ nᵀn 是区域内法向量的二阶矩矩阵。λ_min 反映了法向量在三个主轴方向的分散程度：
- 单一平面 → S 秩为 1，λ_min ≈ 0
- 墙角（正交平面）→ S 秩为 2，λ_min 中等
- 复杂三面角 → S 秩为 3，λ_min 最大

λ_min 越大 → 几何结构越丰富 → ICP 约束越完整。

**λ_min 不反映覆盖密度**。引入 N_effective：

```
score = λ_min × N_effective - a·pitch + b·f(δyaw)
```

其中 **N_effective = 矩形内命中体素的像素数**（无论该体素的 n 是否为 0）。

二者无重复计算：
- n=0 的体素 → n×nᵀ = 0 矩阵 → 对 S 无贡献 → 不会虚增 λ_min
- 但该像素计入 N_effective → 反映几何覆盖密度（"这个方向有东西"）
- N_effective 测覆盖，λ_min 测多样性，各司其职

### B. Octomap vs Dense Grid 的 ERP Ray-cast

ERP 对每个像素方向发射射线，用 ray-march 找到第一个被占据的体素。

**Dense 3D Array**：
- DDA 每步 O(1)（整数加法 + 数组索引）
- 每条射线最多 D/s 步（50m / 1m = 50 步）
- 但无论体素是否被占据，每步必走——户外场景 >95% 是空气/天空，大量步数浪费

**Octomap（推荐）**：
- 层次结构允许跳跃空区域：如果一个大节点为空，整块跳过
- 户外稀疏场景下，每条射线平均只需 log(D/s) ≈ 5-6 次树遍历
- 150m 范围对 Dense 不可行（~27M 体素），对 Octomap 完全可行

| | Dense Grid | Octomap |
|---|---|---|
| 单步开销 | O(1)，整数运算 | O(log N)，指针 |
| 空旷区域 | 每步必走 | O(1) 跳过 |
| 内存(150m, 1m) | ~27M 体素 (不可行) | 与占据数成正比 (可行) |
| 总步数(估算) | ~10M+ | ~0.3-0.5M (空洞跳跃) |
| 实现复杂度 | 低 | 中 |

**结论**：采用 Octomap，户外稀疏场景下 ray-cast 效率显著更高。

### C. Score 函数

```
score = λ_min × N_effective - a·pitch + b·f(δyaw)
```

- `λ_min × N_effective`：可定位性 × 有效覆盖
- `-a·pitch`：pitch 越小（越向上看），分越高。向上看减少遮挡。**注意负号**
- `b·f(δyaw)`：f 单调递减，δyaw 越小 f 越大（奖励平滑）。b > 0

**f(δyaw) 设计**：
- 带死区 [0, δ_deadzone]：f 恒定（不惩罚小角度变化），避免局限于附近——LiDAR 点云预处理有去畸变，合理的 δyaw 不影响数据质量
- 超过死区后单调递减，如：

```
f(δyaw) = 1                          , if δyaw ≤ δ_deadzone
f(δyaw) = exp(-(δyaw - δ_deadzone) / τ), if δyaw > δ_deadzone
```

### D. 时间一致性和车辆运动模型

暂时不考虑（后续迭代加入）。

### E. IEKF 协方差方向加权

有道理，但暂时不考虑（先用统一的 λ_min 跑通基本流程）。

### F. 积分图的水平扩展

ERP 水平方向 360°，Avia FoV 矩形（水平 ~70.4°）在 yaw 接近 0°/360° 边界时会跨越边界，积分图不支持跨边界矩形查询。

**解决方案**：水平方向扩展。构建积分图时，在 ERP 右侧额外复制左侧 FoV_width 列：

```
原始 ERP 宽度: W_h = 360° / resolution
扩展后宽度: W_h + FoV_width_pixels
```

扩展后任何中心 yaw 对应的 FoV 矩形都不会跨边界，可在积分图上做常规 O(1) 查询。

### G. 动态物体和季节变化

- 树叶等不稳定特征：构建体素地图时，法向量不一致的体素 → 标记 n=(0,0,0) → 对 S 无贡献 → 自然成为"不好的体素"
- 遮挡：-a·pitch 项使策略倾向向上看，减少近处遮挡

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
│  ├── 每个节点: 聚合法向量, 检查一致性                              │
│  │    - 一致 → n_mean                                             │
│  │    - 不一致 → (0,0,0) (树叶、边缘等)                          │
│  └── 输出: VoxelOctomap (序列化存储, 可快速加载)                  │
│                                                                  │
├──────────────────────────────────────────────────────────────────┤
│                  ONLINE: 策略更新循环 (2-5 Hz)                    │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ① getCurrentPose()                                              │
│     └── 订阅 /localization/estimation                            │
│                                                                  │
│  ② ERPProjector::project(octomap, pose, params)                  │
│     ├── 以当前位姿为球心                                          │
│     ├── 范围: 半径 150m (Octomap 空洞跳跃, 远距几乎不增开销)     │
│     ├── 垂直范围: ~140° (以覆盖 pitch_candidate 范围)             │
│     │     pitch ∈ [-50°, 20°] 对应 FoV 矩形上下沿 ≈              │
│     │     [-88.5°, 58.5°], 取 ERP 垂直 [-90°, 50°] ≈ 140°       │
│     ├── 水平范围: 360° + FoV_width 扩展                          │
│     ├── 分辨率: ~1°/pixel                                        │
│     ├── 每像素: octree ray-cast → 第一个命中体素的法向量 n      │
│     │    无命中 → n = (0,0,0)                                    │
│     └── 输出: ERPDepthImage[n_row][n_col] = n ∈ R³ + occupied_flag│
│                                                                  │
│  ③ IntegralImage::build(erp_image)                               │
│     ├── 每个 pixel: M = n * n^T (3x3 对称, 6 独立分量)           │
│     ├── 每个 pixel: occupied (0/1)                                │
│     ├── 构建 6 个矩阵分量积分图 + 1 个 occupied 计数积分图         │
│     └── 总计 7 个标量积分图                                       │
│                                                                  │
│  ④ RectangleSearch::search(integral_imgs, curr_yaw, curr_pitch)  │
│     ├── Avia FoV 矩形: w = 70.4°/res, h = 77.2°/res              │
│     ├── 候选 (yaw_c, pitch_c):                                    │
│     │   - yaw_c   ∈ [-180°, 180°), step 2-5°                     │
│     │   - pitch_c ∈ [-50°, 20°],    step 2-5°                    │
│     ├── 每个候选 O(1):                                            │
│     │   a) 积分图查 S (6 分量)                                    │
│     │   b) 积分图查 N_effective                                   │
│     │   c) Eigen::SelfAdjointEigenSolver<3> → λ_min              │
│     │   d) δyaw = shortest_angle(yaw_c, curr_yaw)                │
│     │   e) score = λ_min × N_effective - a·pitch_c + b·f(δyaw)  │
│     └── argmax → best_yaw, best_pitch                            │
│                                                                  │
│  ⑤ publishGimbalCmd(best_yaw, best_pitch)                        │
│     └── cyber_msgs::GimbalCommand → /gimbal_cmd                  │
│         cmd=PAN,  data=best_yaw (度)                             │
│         cmd=TILT, data=best_pitch (度)                           │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

**ERP 水平扩展示意**：
```
原始 ERP (360°):
┌────────────────────────────────────────────┐
│ 0°  ...                                    │ 360°
└────────────────────────────────────────────┘

扩展后 (360° + FoV_width):
┌────────────────────────────────────────────┬──────────────────┐
│ 0°  ...                                    │ 360° │ 0°..FoV_w │
└────────────────────────────────────────────┴──────────────────┘
        ◄─── FoV 矩形(跨边界时) ───►
              在扩展区完成查询, 无需 wraparound
```

---

## 三、目录/文件结构

```
voxel_motion_strategy/
├── include/voxel_motion_strategy/
│   ├── octomap_builder.h     # Octomap 构建 + 法向量一致性检查
│   ├── erp_projector.h       # ERP 投影 + octree ray-cast
│   ├── integral_image.h      # 3×3 矩阵积分图 + 占用计数积分图
│   └── rectangle_search.h    # 候选搜索 + score 计算
├── src/
│   ├── voxel_motion_strategy_node.cpp  # ROS 主节点
│   ├── octomap_builder.cpp
│   ├── erp_projector.cpp
│   ├── integral_image.cpp
│   ├── rectangle_search.cpp
│   └── octomap_test.cpp      # 测试: 可视化体素法向量一致性
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
| `erp_range_max` | 150 m | 射线最大距离 (Octomap 空洞跳跃, 远距开销小) |
| `erp_vfov_min` | -90° | ERP 垂直下界 |
| `erp_vfov_max` | 50° | ERP 垂直上界 (覆盖 pitch_candidate 的 FoV) |
| `pitch_candidate_min` | -50° | 候选 pitch 下界（向上看） |
| `pitch_candidate_max` | 20° | 候选 pitch 上界 |
| `yaw_step` | 3° | 候选 yaw 搜索步长 |
| `pitch_step` | 3° | 候选 pitch 搜索步长 |
| `voxel_normal_consistency_threshold` | 待定 | 法向量一致性阈值 (从测试确定) |
| `fov_horizontal` | 70.4° | Avia 水平 FoV |
| `fov_vertical` | 77.2° | Avia 垂直 FoV |
| `weight_pitch` (a) | 待调 | pitch 惩罚权重 |
| `weight_dyaw` (b) | 待调 | δyaw 奖励权重 |
| `dyaw_deadzone` | 待调 | δyaw 死区角度 |
| `strategy_update_rate` | 4 Hz | 策略更新频率 |

---

## 五、实施计划

### Step 0: 体素地图测试

先写 `octomap_test.cpp`：
- 加载全局地图和法向量
- 构建 Octomap
- 统计/输出每个体素的法向量不一致程度（mean angular deviation 分布）
- 根据实际数据确定 `voxel_normal_consistency_threshold`
- 验证树叶等不稳定区域是否被正确标记为 n=(0,0,0)

### Step 1: 核心模块实现（按依赖顺序）

1. `octomap_builder` — Octomap 构建 + 空间查询接口
2. `erp_projector` — Octree ray-cast + ERP 深度图生成
3. `integral_image` — 7 个标量积分图（6 个矩阵分量 + 1 个占用计数）
4. `rectangle_search` — 候选搜索 + score 计算

### Step 2: ROS 节点集成

- 订阅 `/localization/estimation`
- 发布 `/gimbal_cmd`
- 参数从 YAML 加载

### Step 3: 调参验证

- 用 rosbag 回放测试各参数组合
- 与 simplecount/RL 分支效果对比
