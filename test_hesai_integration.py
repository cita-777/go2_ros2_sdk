#!/usr/bin/env python3
"""
Hesai LiDAR 集成测试脚本

此脚本用于验证 Hesai LiDAR 是否正确集成到 Go2 机器人系统中。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Header
import time
import sys

class HesaiIntegrationTest(Node):
    def __init__(self):
        super().__init__('hesai_integration_test')
        
        self.get_logger().info("开始 Hesai LiDAR 集成测试...")
        
        # 测试状态
        self.point_cloud_received = False
        self.laser_scan_received = False
        self.test_start_time = time.time()
        self.test_timeout = 30.0  # 30秒超时
        
        # 订阅主题
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud2',
            self.point_cloud_callback,
            10
        )
        
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )
        
        # 创建定时器检查测试状态
        self.timer = self.create_timer(1.0, self.check_test_status)
        
        self.get_logger().info("等待激光雷达数据...")
    
    def point_cloud_callback(self, msg):
        """点云数据回调"""
        if not self.point_cloud_received:
            self.get_logger().info("✓ 接收到点云数据")
            self.get_logger().info(f"  - Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"  - 点数量: {msg.width * msg.height}")
            self.get_logger().info(f"  - 时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            self.point_cloud_received = True
    
    def laser_scan_callback(self, msg):
        """激光扫描数据回调"""
        if not self.laser_scan_received:
            self.get_logger().info("✓ 接收到激光扫描数据")
            self.get_logger().info(f"  - Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"  - 角度范围: {msg.angle_min:.2f} 到 {msg.angle_max:.2f} 弧度")
            self.get_logger().info(f"  - 角度增量: {msg.angle_increment:.4f} 弧度")
            self.get_logger().info(f"  - 距离范围: {msg.range_min:.2f} 到 {msg.range_max:.2f} 米")
            self.get_logger().info(f"  - 扫描点数: {len(msg.ranges)}")
            self.laser_scan_received = True
    
    def check_test_status(self):
        """检查测试状态"""
        elapsed_time = time.time() - self.test_start_time
        
        if self.point_cloud_received and self.laser_scan_received:
            self.get_logger().info("🎉 所有测试通过！Hesai LiDAR 集成成功。")
            self.print_test_summary(True)
            rclpy.shutdown()
            return
        
        if elapsed_time > self.test_timeout:
            self.get_logger().error("❌ 测试超时！")
            self.print_test_summary(False)
            rclpy.shutdown()
            return
        
        # 显示进度
        remaining_time = self.test_timeout - elapsed_time
        self.get_logger().info(f"测试进行中... 剩余时间: {remaining_time:.1f}秒")
    
    def print_test_summary(self, success):
        """打印测试摘要"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Hesai LiDAR 集成测试摘要")
        self.get_logger().info("=" * 50)
        
        status = "✓" if self.point_cloud_received else "❌"
        self.get_logger().info(f"{status} 点云数据接收: {'成功' if self.point_cloud_received else '失败'}")
        
        status = "✓" if self.laser_scan_received else "❌"
        self.get_logger().info(f"{status} 激光扫描数据接收: {'成功' if self.laser_scan_received else '失败'}")
        
        self.get_logger().info("=" * 50)
        
        if success:
            self.get_logger().info("🎉 集成测试成功！Hesai LiDAR 正常工作。")
        else:
            self.get_logger().error("❌ 集成测试失败！请检查配置和连接。")
            self.print_troubleshooting_tips()
    
    def print_troubleshooting_tips(self):
        """打印故障排除提示"""
        self.get_logger().info("\n故障排除提示:")
        self.get_logger().info("1. 检查 LIDAR_TYPE 环境变量是否设置为 'hesai'")
        self.get_logger().info("2. 验证 Hesai LiDAR 硬件连接和电源")
        self.get_logger().info("3. 检查网络配置和 IP 地址设置")
        self.get_logger().info("4. 确认 hesai_ros_driver 节点是否正在运行:")
        self.get_logger().info("   ros2 node list | grep hesai")
        self.get_logger().info("5. 检查主题是否发布:")
        self.get_logger().info("   ros2 topic list | grep point_cloud2")
        self.get_logger().info("6. 查看节点日志:")
        self.get_logger().info("   ros2 log")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = HesaiIntegrationTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
