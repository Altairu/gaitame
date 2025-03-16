#!/bin/bash
# ROS2環境のセットアップ
echo "Sourcing ROS2 setup.bash"
source /opt/ros/humble/setup.bash

# colconビルド
echo "Running colcon build"
colcon build --symlink-install

# ビルドの結果を確認
if [ $? -ne 0 ]; then
    echo "colcon build failed"
    exit 1
fi

# ワークスペース環境のセットアップ
echo "Sourcing workspace setup.bash"
source install/setup.bash

# ノードを同時に起動 (launchディレクトリ内のlaunchファイル)
echo "Launching nodes"
ros2 launch launch/gaitame_launch.py
