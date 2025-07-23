#!/bin/bash

# Hesai LiDAR 集成构建和测试脚本
# 此脚本自动化构建过程并运行基本测试

set -e  # 遇到错误时退出

echo "=========================================="
echo "Hesai LiDAR 集成构建和测试脚本"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 函数：打印彩色消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查 ROS2 环境
check_ros2_environment() {
    print_info "检查 ROS2 环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 环境未设置。请运行: source /opt/ros/\$ROS_DISTRO/setup.bash"
        exit 1
    fi
    
    print_info "ROS2 发行版: $ROS_DISTRO"
    
    # 检查必要的工具
    if ! command -v colcon &> /dev/null; then
        print_error "colcon 未安装。请安装: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
}

# 安装依赖项
install_dependencies() {
    print_info "检查和安装依赖项..."
    
    # 检查是否需要 sudo
    if [ "$EUID" -ne 0 ]; then
        SUDO="sudo"
    else
        SUDO=""
    fi
    
    # 安装 Boost
    if ! dpkg -l | grep -q libboost-all-dev; then
        print_info "安装 Boost 库..."
        $SUDO apt-get update
        $SUDO apt-get install -y libboost-all-dev
    else
        print_info "Boost 库已安装"
    fi
    
    # 安装 YAML
    if ! dpkg -l | grep -q libyaml-cpp-dev; then
        print_info "安装 YAML 库..."
        $SUDO apt-get install -y libyaml-cpp-dev
    else
        print_info "YAML 库已安装"
    fi
    
    # 安装 ROS2 包
    print_info "安装 ROS2 依赖包..."
    $SUDO apt install -y \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-std-msgs \
        ros-$ROS_DISTRO-pointcloud-to-laserscan \
        ros-$ROS_DISTRO-ament-cmake \
        ros-$ROS_DISTRO-rclcpp \
        ros-$ROS_DISTRO-rclcpp-action
}

# 构建系统
build_system() {
    print_info "构建 Hesai LiDAR 驱动..."
    
    # 确保在正确的目录中
    if [ ! -f "src/hesai_lidar_ws/package.xml" ]; then
        print_error "未找到 Hesai LiDAR 包。请确保在正确的工作区目录中运行此脚本。"
        exit 1
    fi
    
    # 运行 rosdep
    print_info "安装 ROS 依赖项..."
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    
    # 构建
    print_info "开始构建..."
    colcon build --packages-select hesai_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_info "构建成功！"
    else
        print_error "构建失败！"
        exit 1
    fi
}

# 运行基本测试
run_basic_tests() {
    print_info "运行基本测试..."
    
    # 源化环境
    source install/setup.bash
    
    # 检查包是否正确安装
    if ros2 pkg list | grep -q hesai_ros_driver; then
        print_info "✓ hesai_ros_driver 包已正确安装"
    else
        print_error "✗ hesai_ros_driver 包未找到"
        return 1
    fi
    
    # 检查可执行文件
    if [ -f "install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node" ]; then
        print_info "✓ hesai_ros_driver_node 可执行文件存在"
    else
        print_error "✗ hesai_ros_driver_node 可执行文件未找到"
        return 1
    fi
    
    # 检查配置文件
    if [ -f "install/hesai_ros_driver/share/hesai_ros_driver/config/go2_config.yaml" ]; then
        print_info "✓ Go2 配置文件存在"
    else
        print_error "✗ Go2 配置文件未找到"
        return 1
    fi
    
    # 检查启动文件
    if [ -f "install/hesai_ros_driver/share/hesai_ros_driver/launch/go2_hesai_lidar.launch.py" ]; then
        print_info "✓ Go2 启动文件存在"
    else
        print_error "✗ Go2 启动文件未找到"
        return 1
    fi
    
    print_info "所有基本测试通过！"
}

# 显示使用说明
show_usage_instructions() {
    print_info "构建完成！使用说明："
    echo ""
    echo "1. 设置环境变量："
    echo "   export ROBOT_IP=\"your_robot_ip\""
    echo "   export CONN_TYPE=\"webrtc\""
    echo "   export LIDAR_TYPE=\"hesai\""
    echo ""
    echo "2. 启动系统："
    echo "   source install/setup.bash"
    echo "   ros2 launch go2_robot_sdk robot.launch.py"
    echo ""
    echo "3. 运行集成测试："
    echo "   python3 test_hesai_integration.py"
    echo ""
    echo "4. 查看详细文档："
    echo "   cat HESAI_LIDAR_INTEGRATION_GUIDE.md"
}

# 主函数
main() {
    print_info "开始 Hesai LiDAR 集成构建过程..."
    
    check_ros2_environment
    
    # 询问是否安装依赖项
    read -p "是否需要安装系统依赖项？(y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_dependencies
    fi
    
    build_system
    run_basic_tests
    
    print_info "构建和测试完成！"
    show_usage_instructions
}

# 运行主函数
main "$@"
