### Installation dependencies

### Install Boost

    sudo apt-get update
    sudo apt-get install libboost-all-dev

### Install Yaml

    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev

### Compile and run

- ros2

  Create an `src` folder, copy the source code of the ros driver into it, and then run the following command:

  colcon build --symlink-install
  . install/local_setup.bash

  For other ROS2 version

  sudo su
  ros2 launch hesai_ros_driver start.py

### Introduction to the configuration file `config.yaml` parameters

```yaml
lidar:
  - driver:
      use_gpu: false # 是否使用 GPU 加速处理点云数据
      source_type: 1 # 数据源类型：1 表示实时连接激光雷达，2 表示 pcap 文件，3 表示 rosbag 数据包，4 表示串口数据
      # 根据 source_type 的不同，填写对应的配置块（lidar_udp_type, pcap_type, serial_type）

      lidar_udp_type:
        device_ip_address: 192.168.1.201 # 激光雷达设备的 IP 地址（主机接收 IP，为空时使用 UDP 点云的源 IP）
        udp_port: 2368 # UDP 接收端口号
        ptc_port: 9347 # PTC 端口号，用于与激光雷达通信
        multicast_ip_address: 255.255.255.255 # 组播 IP 地址

        use_ptc_connected: true # 是否使用 PTC 连接（设为 false 表示不使用）
        correction_file_path: "Your correction file path" # 校正文件的路径
        firetimes_path: "Your firetime file path" # 点火时间文件的路径

        use_someip: false # 是否使用 SOME/IP 协议
        host_ip_address: "" # 主机 IP 地址（当使用 SOME/IP 时配置）
        fault_message_port: 0 # 故障信息端口号

        standby_mode: -1 # 待机模式：-1 表示无效，0 表示正常运行，1 表示待机
        speed: -1 # 转速：-1 表示无效，需确保设置的值是当前激光雷达支持的
        ptc_mode: 0 # PTC 通信模式：0 表示 TCP，1 表示 TCP_SSL
        # 使用 TCP_SSL 时的配置
        certFile: "" # 用户证书文件路径
        privateKeyFile: "" # 用户私钥文件路径
        caFile: "" # CA 证书文件路径

      pcap_type:
        pcap_path: "Your pcap file" # pcap 文件路径
        correction_file_path: "Your correction file path" # 校正文件路径
        firetimes_path: "Your firetime file path" # 点火时间文件路径

        pcap_play_synchronization: true # 是否同步播放 pcap 数据（与主机时间同步）
        pcap_play_in_loop: false # 是否循环播放 pcap 文件

      rosbag_type:
        correction_file_path: "Your correction file path" # 校正文件路径
        firetimes_path: "Your firetime file path" # 点火时间文件路径

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud" # 接收点云数据的串口名称（如使用 JT16）
        rs232_com: "Your serial port name for sending cmd" # 发送指令的串口名称
        point_cloud_baudrate: 3125000 # 点云波特率
        correction_save_path: "" # 校正文件保存路径（用于存储激光雷达角度校正文件）
        correction_file_path: "Your correction file path" # 校正文件路径

      # 公共模块配置
      use_timestamp_type: 0 # 时间戳类型：0 表示使用点云自带时间戳，1 表示使用接收时间戳
      frame_start_azimuth: 0 # 帧起始方位角（适用于 Pandar128），范围 1~359，小于 0 表示不使用
      # 坐标变换参数
      transform_flag: false # 是否启用坐标变换
      x: 0 # X 轴偏移量（单位：米）
      y: 0 # Y 轴偏移量（单位：米）
      z: 0 # Z 轴偏移量（单位：米）
      roll: 0 # Roll 角度（单位：度）
      pitch: 0 # Pitch 角度（单位：度）
      yaw: 0 # Yaw 角度（单位：度）

      # 视场角配置，范围 [1, 359]，[-1, -1] 表示使用默认值
      fov_start: -1 # 起始视场角
      fov_end: -1 # 结束视场角

      # 其他配置
      enable_packet_loss_tool: true # 是否启用 UDP 丢包检测工具
      distance_correction_flag: false # 是否启用光学中心校正
      xt_spot_correction: false # 是否启用 XT S 点云分层校正
      device_udp_src_port: 0 # 用于过滤特定源端口的点云数据（适用于多个雷达场景），设置值 >= 1024
      device_fault_port: 0 # 用于过滤特定源端口的故障信息（适用于多个雷达场景），设置值 >= 1024

    ros:
      ros_frame_id: hesai_lidar # ROS 消息的 frame_id，用于标识点云和数据包来源
      ros_recv_packet_topic: /lidar_packets # 用于从 rosbag 接收原始数据包的主题
      ros_send_packet_topic: /lidar_packets # 用于通过 ROS 发送原始数据包的主题
      ros_send_point_cloud_topic: /lidar_points # 用于通过 ROS 发送点云数据的主题
      ros_send_imu_topic: /lidar_imu # 用于通过 ROS 发送 IMU 数据的主题
      ros_send_packet_loss_topic: /lidar_packets_loss # 用于监控丢包情况的主题
      # ros_recv_correction_topic: /lidar_corrections # 用于从 rosbag 接收校正文件的主题（已注释）
      # ros_send_correction_topic: /lidar_corrections # 用于通过 ROS 发送校正文件的主题（已注释）
      # ros_send_firetime_topic: /lidar_firetime # 用于发送点火时间的主题（已注释）
      # ros_send_ptp_topic: /lidar_ptp # 用于发送 PTP 锁定状态和偏移量的主题（已注释）
      send_packet_ros: false # 是否通过 ROS 发送原始数据包
      send_point_cloud_ros: true # 是否通过 ROS 发送点云数据
      send_imu_ros: true # 是否通过 ROS 发送 IMU 数据
```

### Real time playback

In the configuration file, set `source_type` to `1`, then configure the parameters under `lidar_udp_type`. Generally, you only need to configure `device_ip_address`, `udp_port`, and `ptc_port`. If the point cloud destination IP is multicast, you need to configure `device_ip_address`. It is recommended to configure `correction_file_path` to prevent point cloud parsing failures when the lidar angle calibration file acquisition fails. Then run start.launch.

### Parsing PCAP file

In the configuration file, set `source_type` to `2`, then configure the parameters under `pcap_type`. Generally, you need to configure `pcap_path`, `correction_file_path`, and `firetime_file_path`. For detailed information about `pcap_play_synchronization` and `pcap_play_in_loop` functionality, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### Record and playback ROSBAG file

- Record ：

  When playing or parsing PCAP in real-time, set `send_packet_ros` to `true`, start another terminal and enter the following command to record the data packet ROSBAG.

  rosbag record ros_send_packet_topic
- Playback ：

  First, replay the recorded rosbag file `test.bag` using the following command.

  rosbag play test.bag

  Set the `source_type` in the configuration file to `3`, then configure the parameters under `rosbag_type`. Generally, you need to configure `correction_file_path` , `firetime_file_path` and `ros_recv_packet_topic`(the topic name of rosbag, under `ros`), then run start.launch.

### Parsing serial data

In the configuration file, set `source_type` to `4`, then configure the parameters under `serial_type`. Generally, you need to configure `rs485_com` and `rs232_com`. It is recommended to configure `correction_file_path` (required if `rs232_com` is not used). For other parameters, please refer to the parameter introduction section in the SDK README. Then run start.launch.

### Realize multi lidar fusion

According to the configuration of a single lidar, multiple drivers can be created in `config.yaml`, as shown in the following example

```yaml
lidar:
  - driver:
      use_gpu: false
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial  
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.201              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2368                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        use_someip: false
        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
  
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
  
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it  
      #transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      #fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
    ros:
      ros_frame_id: hesai_lidar                       # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                          # true: Send packets through ROS 
      send_point_cloud_ros: true                      # true: Send point cloud through ROS  
      send_imu_ros: true                              # true: Send imu through ROS  
  - driver:
      use_gpu: false
      source_type: 1                                  # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial  
      # Depending on the type of source_type, fill in the corresponding configuration block (lidar_udp_type, pcap_type, serial_type)
      lidar_udp_type:
        device_ip_address: 192.168.1.202              # host_ip_address. If empty(""), the source ip of the udp point cloud is used
        udp_port: 2369                                # UDP destination port
        ptc_port: 9347                                # PTC port of lidar
        multicast_ip_address: 255.255.255.255

        use_ptc_connected: true                       # Set to false when ptc connection is not used
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        use_someip: false
        host_ip_address: ""
        fault_message_port: 0

        standby_mode: -1                              # The standby mode: [-1] is invalit [0] in operation [1] standby
        speed: -1                                     # The speed: [-1] invalit, you must make sure your set has been supported by the lidar you are using
        ptc_mode: 0                                   # The ptc mode: [0] tcp [1] tcp_ssl
        # tcp_ssl use
        certFile: ""                                  # Represents the path of the user's certificate
        privateKeyFile: ""                            # Represents the path of the user's private key 
        caFile: ""                                    # Represents the path of the CA certificate 

      pcap_type:
        pcap_path: "Your pcap file"                         # The path of pcap file
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

        pcap_play_synchronization: true                     # pcap play rate synchronize with the host time
        pcap_play_in_loop: false
  
      rosbag_type:
        correction_file_path: "Your correction file path"   # The path of correction file
        firetimes_path: "Your firetime file path"           # The path of firetimes file

      serial_type:
        rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
        rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
        point_cloud_baudrate: 3125000
        correction_save_path: ""                                      # turn on when you need to store angle calibration files(from lidar)
        correction_file_path: "Your correction file path"             # The path of correction file
  
      # public module
      use_timestamp_type: 0                 # 0 use point cloud timestamp; 1 use receive timestamp
      frame_start_azimuth: 0                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it  
      #transform param
      transform_flag: false
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
      #fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
      fov_start: -1
      fov_end:  -1
      # other config
      enable_packet_loss_tool: true         # enable the udp packet loss detection tool
      distance_correction_flag: false       # set to true when optical centre correction needs to be turned on
      xt_spot_correction: false             # Set to TRUE when XT S point cloud layering correction is required
      device_udp_src_port: 0                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
      device_fault_port: 0                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
    ros:
      ros_frame_id: hesai_lidar                         # Frame id of packet message and point cloud message
      ros_recv_packet_topic: /lidar_packets_2           # Topic used to receive lidar packets from rosbag
      ros_send_packet_topic: /lidar_packets_2           # Topic used to send lidar raw packets through ROS
      ros_send_point_cloud_topic: /lidar_points_2       # Topic used to send point cloud through ROS
      ros_send_imu_topic: /lidar_imu_2                  # Topic used to send lidar imu message
      ros_send_packet_loss_topic: /lidar_packets_loss_2 # Topic used to monitor packets loss condition through ROS
      send_packet_ros: false                            # true: Send packets through ROS 
      send_point_cloud_ros: true                        # true: Send point cloud through ROS  
      send_imu_ros: true                                # true: Send imu through ROS  
```
