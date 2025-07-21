下载

git clone --recurse-submodules https://github.com/cita-777/go2_ros2_sdk.git

降级numpy

pip install "numpy<2"

编译

colcon build --symlink-install

source install/setup.bash

运行

ros2 launch go2_robot_sdk robot.launch.py
