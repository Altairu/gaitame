# Gaitame プロジェクト README

## 概要
このプロジェクトは、Spresense ボードと CXD5602PWBIMU センサーを使用して姿勢推定を行い、そのデータを ROS2 を通じて配信・可視化します
マイコン用コードは [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) リポジトリから取得し、必要に応じてカスタマイズしています。

## フォルダ構成
- `gaitame/gaitame/`
  - マイコン用プログラム (Arduino IDE 用) の `gaitame.ino`
- `gaitame/src/gaitamepkg/gaitamepkg/`
  - ROS2 ノード用プログラム
    - `serialnode.py`: シリアル通信でマイコンからIMUデータと計算済み姿勢データを受信し、PoseStamped および TF で配信する
    - `rviznode.py`: PoseStamped メッセージを受信し、RViz 上で姿勢をテキスト表示する

## システムの動作概要
- **マイコン側 (gaitame.ino):**  
   [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino)) をベースに、Spresense と接続された CXD5602PWBIMU センサーから加速度・ジャイロデータを取得し、補正・姿勢推定を行います。
- **ROS2 ノード (serialnode.py, rviznode.py):**  
  シリアル通信を通じてマイコンから受信した計算済み姿勢データ（roll, pitch, yaw〔度〕）をクォータニオンに変換し、`imu_pose` トピックに配信します。  
  次に、`rviznode.py` がこの `imu_pose` トピックを購読し、Marker で姿勢情報（オイラー角）を RViz 上に表示します。

## 導入方法
   
### ROS2 ノードのセットアップ
1. **依存関係のインストール:**  
   必要な ROS2 パッケージをインストールしてください。
2. **ノードの起動:**  
   `serialnode.py` と `rviznode.py` をそれぞれ起動します。
なお，`run_all.sh`ですべて行うことができます．
```bash
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

```

## 謝辞
[cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) のMasaaki Hijikataさんに深く感謝いたします。  

## ライセンス
このプロジェクトは MIT ライセンスの下で提供されています。
