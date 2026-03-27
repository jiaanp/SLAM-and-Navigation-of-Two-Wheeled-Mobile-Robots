# SLAM and Navigation of Two-Wheeled Mobile Robots

这是一个面向两轮差速小车的学习与实践项目，包含：

- 基于 `ESP32` 的底盘控制与传感器读取示例
- 基于 `micro-ROS` 的板端到 ROS 2 通信
- 基于 `ROS 2 Humble` 的建图、导航与 TF 组织
- 基于编码器里程计和 `IMU` 的 `EKF` 融合链路

项目当前已经打通以下核心数据流：

- 小车底层发布 `/odom`
- 小车底层发布 `/imu/data`
- `robot_localization/ekf_node` 融合 `/odom + /imu/data`
- 输出 `/odometry/filtered`

## 项目结构

仓库主要内容位于：

- `SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/`

其下包含两个主要部分。

### 1. ESP32 / PlatformIO 示例

- `example_motion_ctl`
  差速底盘主控制示例，包含电机闭环、编码器、`micro-ROS`、`odom` 发布、`IMU` 发布
- `example_IMU`
  MPU6050 单独读取示例，用于验证 I2C 连线和 IMU 数据
- `example_led`
  LED 示例
- `example_ultrasound`
  超声波示例
- `example_project`
  PlatformIO 默认示例工程

### 2. ROS 2 工作区

- `fishbot_ws`

工作区源码位于：

- `SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws/src`

当前包含的主要 ROS 2 包有：

- `fishbot_bringup`
  启动文件、EKF 配置、`odom2tf`
- `robot_description`
  机器人 URDF
- `robot_navigation2`
  Navigation2 相关配置
- `ydlidar_ros2`
  雷达驱动
- `ros_serial2wifi`
  TCP 转发工具
- `micro_ros_msgs`
  自定义 micro-ROS 消息包

## 当前实现概览

### 底盘控制

底盘主程序位于：

- `SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/example_motion_ctl/src/main.cpp`

当前实现包括：

- 订阅 `cmd_vel`
- 差速逆运动学
- 左右轮 `PID` 速度闭环
- 编码器里程计计算与 `odom` 发布
- `MPU6050` 数据读取与 `/imu/data` 发布

### IMU 融合

`IMU` 数据由小车端发布为：

- `/imu/data`

里程计由小车端发布为：

- `/odom`

融合由 ROS 2 侧 `robot_localization` 完成，配置文件位于：

- `SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws/src/bring_up/config/ekf.yaml`

融合结果输出为：

- `/odometry/filtered`

### Bringup 启动链

主启动文件位于：

- `SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws/src/bring_up/launch/bringup.launch.py`

默认启动：

- `micro_ros_agent`
- `robot_state_publisher`
- `ekf_node`
- `odom2tf`

可选启动：

- `ydlidar`
- `ros_serial2wifi tcp_server`

## 环境要求

建议环境：

- Ubuntu 22.04
- ROS 2 Humble
- PlatformIO
- ESP32 开发板
- MPU6050
- 编码器、电机驱动、两轮差速底盘

系统侧需要具备：

- `robot_localization`
- `micro_ros_agent`
- `nav2`
- `rviz2`

## 构建方法

### 1. 构建 ROS 2 工作区

```bash
cd ~/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 烧录 ESP32 底盘程序

进入底盘工程目录：

```bash
cd ~/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/example_motion_ctl
pio run
pio run -t upload
```

### 3. 单独验证 IMU 示例

```bash
cd ~/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/example_IMU
pio run
pio run -t upload
pio device monitor -b 115200
```

## 运行方法

### 一键启动基础链路

```bash
cd ~/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/SLAM-and-Navigation-of-Two-Wheeled-Mobile-Robots/fishbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fishbot_bringup bringup.launch.py
```

### 启动时同时开启雷达和 TCP Server

```bash
ros2 launch fishbot_bringup bringup.launch.py enable_lidar:=true enable_tcp_server:=true
```

## 常用验证命令

### 查看基础话题

```bash
ros2 topic list | grep -E "odom|imu"
```

### 查看小车发布的 IMU

```bash
ros2 topic echo /imu/data --once
```

### 查看编码器里程计

```bash
ros2 topic echo /odom --once
```

### 查看融合后的里程计

```bash
ros2 topic echo /odometry/filtered --once
```

## 关键说明

### 1. `/imu/data` 由谁发布

`/imu/data` 不是 PC 端节点生成的，而是 `ESP32` 底盘固件通过 `micro-ROS` 发布的。

### 2. 卡尔曼滤波体现在哪里

本项目当前没有在 `ESP32` 端单独手写卡尔曼滤波器。

卡尔曼滤波体现在：

- `robot_localization` 的 `ekf_node`
- `fishbot_ws/src/bring_up/config/ekf.yaml`

也就是说，现在的融合逻辑是：

- 小车端发布原始 `odom`
- 小车端发布原始 `imu`
- ROS 2 侧用 `EKF` 做融合

### 3. 单位约定

ROS 标准里：

- 位置使用 `m`
- 线速度使用 `m/s`
- 角度使用 `rad`
- 角速度使用 `rad/s`

如果后续继续调整底盘固件，建议确保对外发布的 ROS 消息遵循以上单位。

## 当前已知问题

- 工作区中可能存在历史遗留包目录或旧安装产物，需要在构建前清理
- `ydlidar` 和 `tcp_server` 在某些环境下可能因为端口或设备占用导致启动失败
- `EKF` 对输入数据质量较敏感，协方差和量纲配置需要结合实机继续调优

## 后续可扩展方向

- 修正并统一 `odom` 对外发布单位
- 完善 `Odometry` 与 `Imu` 的 covariance
- 优化 `EKF` 参数，降低漂移与抖动
- 接入 Navigation2 完整导航链路
- 增加地图构建与定位流程说明

## 说明

本项目目前更偏向学习、调试和逐步集成的工程仓库，而不是一个完全收敛的产品化仓库。建议在实机联调时优先逐段验证：

1. 先验证小车底层是否能稳定发布 `/odom`
2. 再验证 `/imu/data`
3. 再验证 `/odometry/filtered`
4. 最后接入雷达、建图和导航
