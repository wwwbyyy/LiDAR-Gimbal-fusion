<style>
img{
    width: 60%;
    padding-left: 20%;
}
</style>

# 随动激光雷达算法模块

## 总体框架

硬件系统相关见 [随动云台-激光雷达系统](https://sjtu.feishu.cn/wiki/QZKJwFRObiEUmnkkAoCcdzzKnbh)。

总体框架如下：

<img src="pic/image1.png" alt="总体框架图" style="width:60%; auto; display:block;">

代码中 `iekf3d` 为**定位模块**，包含外部传感器定位、激光雷达定位和两者的融合定位；`lidar` 为**点云预处理模块**；`pelco_control` 为**云台控制和反馈模块**。

## 模块信息

### 定位（iekf3d）

采用基于 IEKF 的紧耦合方法，通过点到面 ICP 计算点云匹配误差，融合激光雷达定位和 IMU、GNSS、轮速计等外部传感器的定位。目前的代码只融合了 IMU 的旋转部分。

### 点云预处理（lidar）

主要用于点云去旋转畸变，同时把点云转换到云台坐标系下。

### 云台（pelco_control）

云台为两轴（yaw、pitch），两轴均可用。可通过指令控制云台，同时会实时反馈云台的 yaw 和 pitch 角。

### 虚拟随动（sim_gimbal_lidar）

接收 360° 激光雷达和虚拟云台指令，输出水平 FoV 为 70.4° 的模拟非重复扫描点云，同时会反馈虚拟云台的 yaw 角。

## 各模块间 ROS 消息的名称和流向

实车项目中的关键 ROS 消息流程如下：

<img src="pic/image2.png" alt="实车 ROS 消息流程图" style="width:60%; auto; display:block;">

虚拟随动的关键 ROS 消息流程如下：

<img src="pic/image3.png" alt="虚拟随动 ROS 消息流程图" style="width:60%; auto; display:block;">

## 开发方法

### 云台指令

云台的两轴均可控制，`pelco_control` 模块会根据 `/gimbal_cmd` 指令进行对应控制。

`/gimbal_cmd` 消息为 `cyber_msgs::GimbalCommand` 类型，需要包含头文件 `GimbalCommand.h`。

`cyber_msgs::GimbalCommand` 类型定义在[cyber_msgs::GimbalCommand定义](common/cyber_msgs/msg/GimbalCommand.msg)。`cmd`表示 pelco-d 协议对应的指令。

可以先定义枚举变量：

```cpp
typedef enum command
{
  STOP = 0x00,        // 停止
  UP = 0x08,          // 上
  DOWN = 0x10,        // 下
  LEFT = 0x04,        // 左
  RIGHT = 0x02,       // 右
  UPLEFT = 0x0C,      // 上左, 12
  UPRIGHT = 0x0A,     // 上右, 10
  DOWNLEFT = 0x14,    // 下左, 20
  DOWNRIGHT = 0x12,   // 下右, 18
  PAN = 0x4B,         // 水平旋转至指定角度, 75
  TILT = 0x4D,        // 俯仰旋转至指定角度, 77
  ASKPAN = 0x51,      // 查询水平位置, 81
  ASKTILT = 0x53      // 查询俯仰位置, 83
};
```

若`cmd`为角度控制（PAN、TILT），`data`是角度制单位的整数；若`cmd`为角速度控制（RIGHT等），`data`是0-63范围内的整数。

云台反馈消息 `/pan` `/tilt` 为 `std_msgs::Float64MultiArray` 类型，`data[0]` 是时间戳， `data[1]` 是对应角度制的角度。

### 虚拟随动

只能控制yaw角，可以按角度控制或按角速度控制。控制消息为 `std_msgs::Float64MultiArray` 类型， `data[0]` 是时间戳， `data[1]` 是指令数据（角度制）。

控制指令消息的名称在 `config` 文件中配置。

```yaml
lidar_topic: ...  # 360°激光点云
yaw_deg_topic: ...
yaw_deg_angular_velocity_topic: ...
```

反馈消息 `/sim_gimbal_lidar/yaw_deg` 为 `std_msgs::Float64MultiArray` 类型，`data[0]` 是时间戳， `data[1]` 是对应角度制的角度。

### 融合定位

`iekf3d` 的 `config` 文件进行定位相关配置。主要需要修改激光雷达topic和外参。

```yaml
#预处理后的点云
pointcloud2_topic: ...

# 外参，lidar -> body(gps)， [tx, ty, tz, euler_x, euler_y, euler_z]
lidar_extinct: [1.15, 0, 1.96, 0, 0, 1.5315] #hesai，其他激光雷达需要标定
```

`iekf3d` 包括了点云预处理，但是对于非重复扫描雷达目前无法进行畸变校正。

定位结果有两个消息。 `/iekf3d/odometry` 是 ROS 内置的 `nav_msgs::Odometry` 类型。`/localization/estimation` 是自建 `cyber_msgs::LocalizationEstimate` 类型，需要包含头文件`cyber_msgs/LocalizationEstimate.h`，消息定义在[cyber_msgs::LocalizationEstimated定义](common/cyber_msgs/msg/LocalizationEstimate.msg)中，目前加速度和角加速度没有意义。

发布点云消息的含义可以在rviz中查看。

### 运行

**docker相关内容是老项目的**

- load docker image
  
  docker load -i lidar_gimbal_loc.tar

- run docker container
  
  see run_docker.sh

- build
  
  catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

- mapping
  
  see mapping.sh

- localization
  
  see localization.sh

  rviz -d iekf3d/configs/lidar3d.rviz

- lidar node
  
  rosrun lidar lidar_node

- gimbal control node
  
  rosrun pelco_control pelco_control_node

- sim-gimbal-lidar node

  rosrun sim_gimbal_lidar sim_fimbal_lidar_node