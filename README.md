# Gaitame プロジェクト README

## フォルダ構成
- `gaitame/gaitame/`
  - マイコン用プログラム (Arduino IDE 用) の `gaitame.ino`
- `gaitame/src/gaitamepkg/gaitamepkg/`
  - ROS2 ノード用プログラム
    - `serialnode.py`: シリアル通信でマイコンからIMUデータと計算済み姿勢データを受信し、PoseStampedおよびTFで配信する
    - `rviznode.py`: PoseStamped メッセージを受信し、RViz上で姿勢をテキスト表示する

## やっていることの説明
- **マイコン側 (gaitame.ino):**  
  Spresense と接続されたCXD5602PWBIMUセンサーから加速度・ジャイロデータを取得し、補正・姿勢推定を行い、CSV形式で出力します。  
- **ROS2ノード (serialnode.py, rviznode.py):**  
  シリアル通信を通じてマイコンから受信した計算済み姿勢データ（roll, pitch, yaw〔度〕）を使い、クォータニオンに変換して`imu_pose`トピックに配信します。  
  次に、`rviznode.py`がこの`imu_pose`トピックを購読し、Markerで姿勢情報（オイラー角）をRViz上に表示します。

## 導入方法
### Arduino で IMU を使う方法
1. **Spresense 環境のセットアップ:**  
   Spresense の Arduino IDE 用プラグインをインストールし、適切なボード設定を行ってください。
2. **コードの書き込み:**  
   `gaitame/gaitame/gaitame.ino` を Arduino IDE で開き、Spresenseボードに書き込みます。
3. **シリアル通信:**  
   センサーから得られるデータはCSV形式 (1行目: 生データ, 2行目: 姿勢データ) で出力されます。出力されたデータはROS2ノードが受信し処理します。


## RViz2 の使い方
1. **RViz2 の起動:**  
   ターミナルで `rviz2` コマンドを実行します。
2. **Fixed Frame の設定:**  
   Fixed Frame を `base_link` または `imu_link` に設定してください。
3. **トピックの追加:**  
   - 「Add」ボタンから「Pose」ディスプレイを選択し、トピックとして `/imu_pose` を設定します。
   - また、`Marker` ディスプレイを追加すると、`/imu_marker` トピックの情報が表示されます。
4. **確認:**  
   センサからの姿勢情報が正しく表示されることを確認してください。

