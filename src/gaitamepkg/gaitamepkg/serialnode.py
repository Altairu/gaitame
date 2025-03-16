import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import serial
import math
import struct
from tf2_ros import TransformBroadcaster

PACKET_SIZE = 35  # 固定パケットサイズ

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = {}
    q['w'] = cr * cp * cy + sr * sp * sy
    q['x'] = sr * cp * cy - cr * sp * sy
    q['y'] = cr * sp * cy + sr * cp * sy
    q['z'] = cr * cp * sy - sr * sp * cy
    return q

def compute_crc8(data: bytes) -> int:
    crc = 0
    poly = 0x07
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'imu_pose', 10)
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info(f"Opened serial port: {self.ser.name}")
            # ヘッダ行を破棄
            self.ser.readline()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time = self.get_clock().now()
        # タイマー周期を0.05秒から0.005秒に変更して高頻度に呼び出す
        self.create_timer(0.005, self.timer_callback)

    # 新規追加: IMUデータのパブリッシュとTF送信を統一するメソッド
    def publish_imu_data(self, quat, current_time):
        # PoseStampedメッセージのパブリッシュ
        msg = PoseStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'imu_link'
        msg.pose.orientation.x = quat['x']
        msg.pose.orientation.y = quat['y']
        msg.pose.orientation.z = quat['z']
        msg.pose.orientation.w = quat['w']
        self.publisher_.publish(msg)
        # TF変換の送信
        t_msg = TransformStamped()
        t_msg.header.stamp = current_time.to_msg()
        t_msg.header.frame_id = "base_link"
        t_msg.child_frame_id = "imu_link"
        t_msg.transform.translation.x = 0.0
        t_msg.transform.translation.y = 0.0
        t_msg.transform.translation.z = 0.0
        t_msg.transform.rotation.x = quat['x']
        t_msg.transform.rotation.y = quat['y']
        t_msg.transform.rotation.z = quat['z']
        t_msg.transform.rotation.w = quat['w']
        self.tf_broadcaster.sendTransform(t_msg)

    def get_csv_line(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line or any(header in line for header in ["ax", "ay", "az"]):
            return None
        return line

    def timer_callback(self):
        try:
            # 1行目: センサ生データ（6要素）-> 読み捨てまたはログ出力
            sensor_line = self.get_csv_line()
            if sensor_line is None:
                return
            # （オプション: ログ出力）
            self.get_logger().debug(f"Sensor data: {sensor_line}")
            
            # 2行目: 姿勢データ（roll,pitch,yaw: 3要素, 単位: degree）
            orientation_line = self.get_csv_line()
            if orientation_line is None:
                return
            parts = orientation_line.split(',')
            if len(parts) != 3:
                self.get_logger().warn(f"Incomplete orientation data: {orientation_line}")
                return
            # 取得した姿勢は degree なので、ラジアンに変換
            roll_deg, pitch_deg, yaw_deg = map(float, parts)
            roll_rad = roll_deg * (math.pi/180)
            pitch_rad = pitch_deg * (math.pi/180)
            yaw_rad = yaw_deg * (math.pi/180)
            
            current_time = self.get_clock().now()
            quat = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
            self.publish_imu_data(quat, current_time)
            self.get_logger().info(
                f"Orientation: roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°"
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    if node.ser.is_open:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
