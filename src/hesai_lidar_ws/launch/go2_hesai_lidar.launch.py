from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('hesai_ros_driver'),
            'config',
            'go2_config.yaml'
        ),
        description='Path to the Hesai LiDAR configuration file'
    )
    
    # Get the configuration file path
    config_file = LaunchConfiguration('config_file')
    
    # Hesai LiDAR driver node
    hesai_lidar_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_lidar_driver',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Ensure compatibility with Go2 system topics
            ('/lidar_points', '/point_cloud2'),
            ('/lidar_imu', '/hesai_imu'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        hesai_lidar_node,
    ])
