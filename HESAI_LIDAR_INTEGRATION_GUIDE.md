# Hesai LiDAR 集成指南

本指南详细说明如何将 Hesai 激光雷达系统集成到 Go2 机器人 ROS2 工作区中。

## 概述

此集成允许您使用 Hesai LiDAR 替换 Go2 机器人的内置激光雷达系统，同时保持所有现有功能（SLAM、导航、点云处理等）。

## 前提条件

### 系统要求
- Ubuntu 22.04 或兼容系统
- ROS2 Humble/Iron/Rolling
- 已安装的 Go2 ROS2 SDK

### 依赖项安装

```bash
# 安装 Boost 库
sudo apt-get update
sudo apt-get install libboost-all-dev

# 安装 YAML 库
sudo apt-get install -y libyaml-cpp-dev

# 安装其他必要的 ROS2 包
sudo apt install ros-$ROS_DISTRO-sensor-msgs
sudo apt install ros-$ROS_DISTRO-std-msgs
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan
```

## 集成步骤

### 1. 构建系统

```bash
cd /path/to/your/go2_ros2_sdk
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select hesai_ros_driver
source install/setup.bash
```

### 2. 配置 Hesai LiDAR

编辑 `src/hesai_lidar_ws/config/go2_config.yaml` 文件：

```yaml
lidar:
  - driver:
      lidar_udp_type:
        device_ip_address: "YOUR_LIDAR_IP"  # 设置您的激光雷达 IP 地址
        udp_port: 2368                      # 根据您的激光雷达配置调整
        ptc_port: 9347                      # 根据您的激光雷达配置调整
```

### 3. 启动系统

#### 使用 Hesai LiDAR：

```bash
# 设置环境变量
export ROBOT_IP="your_robot_ip"
export CONN_TYPE="webrtc"  # 或 "cyclonedx"
export LIDAR_TYPE="hesai"  # 关键：指定使用 Hesai LiDAR

# 启动系统
ros2 launch go2_robot_sdk robot.launch.py
```

#### 使用内置 LiDAR（原始行为）：

```bash
# 设置环境变量
export ROBOT_IP="your_robot_ip"
export CONN_TYPE="webrtc"
export LIDAR_TYPE="builtin"  # 或者不设置此变量

# 启动系统
ros2 launch go2_robot_sdk robot.launch.py
```

## 验证和测试

### 1. 检查节点状态

```bash
# 检查所有运行的节点
ros2 node list

# 应该看到 hesai_lidar_driver 节点（如果使用 Hesai LiDAR）
```

### 2. 检查主题

```bash
# 检查点云主题
ros2 topic list | grep point_cloud2
ros2 topic echo /point_cloud2 --once

# 检查激光扫描主题
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

### 3. 在 RViz 中验证

启动系统后，RViz 应该自动打开。验证以下内容：

1. **点云数据**：在 RViz 中应该能看到来自 Hesai LiDAR 的点云数据
2. **激光扫描**：应该能看到转换后的 2D 激光扫描数据
3. **机器人模型**：Go2 机器人模型应该正确显示
4. **地图构建**：SLAM 功能应该正常工作

### 4. 测试 SLAM 和导航

1. **SLAM 测试**：
   - 使用手柄控制机器人移动
   - 观察地图是否正确构建
   - 检查点云数据是否准确

2. **导航测试**：
   - 构建完地图后，设置导航目标
   - 验证路径规划和避障功能

## 故障排除

### 常见问题

1. **Hesai LiDAR 节点无法启动**
   - 检查 IP 地址配置
   - 确认激光雷达硬件连接
   - 检查防火墙设置

2. **没有点云数据**
   - 验证网络连接
   - 检查 UDP 端口配置
   - 确认激光雷达电源状态

3. **构建失败**
   - 确保安装了所有依赖项
   - 检查 ROS2 环境是否正确设置
   - 验证 CMakeLists.txt 和 package.xml 配置

### 调试命令

```bash
# 检查 Hesai LiDAR 驱动日志
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args --log-level debug

# 监控网络流量
sudo tcpdump -i any port 2368

# 检查主题频率
ros2 topic hz /point_cloud2
```

## 配置选项

### Hesai LiDAR 配置参数

主要配置文件：`src/hesai_lidar_ws/config/go2_config.yaml`

- `device_ip_address`: 激光雷达 IP 地址
- `udp_port`: UDP 数据端口
- `ptc_port`: PTC 控制端口
- `use_ptc_connected`: 是否使用 PTC 连接
- `transform_flag`: 是否启用坐标变换
- `fov_start/fov_end`: 视场角范围

### 环境变量

- `LIDAR_TYPE`: 设置为 "hesai" 或 "builtin"
- `ROBOT_IP`: Go2 机器人 IP 地址
- `CONN_TYPE`: 连接类型（"webrtc" 或 "cyclonedx"）

## 性能优化

1. **GPU 加速**：在配置文件中设置 `use_gpu: true`（需要 CUDA 支持）
2. **网络优化**：确保激光雷达和主机在同一网段
3. **处理频率**：根据需要调整点云处理频率

## 支持和维护

如果遇到问题，请检查：
1. 系统日志：`journalctl -f`
2. ROS2 日志：`ros2 log`
3. 网络连接状态
4. 硬件连接状态

更多详细信息，请参考：
- Hesai LiDAR 官方文档
- Go2 ROS2 SDK 文档
- ROS2 官方文档
