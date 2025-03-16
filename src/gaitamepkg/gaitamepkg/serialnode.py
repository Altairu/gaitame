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
        # 状態変数：現在のオイラー角 (ラジアン)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
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
            line = self.get_csv_line()
            if line is None:
                return
            parts = line.split(',')
            if len(parts) < 6:
                self.get_logger().warn(f"Incomplete data: {line}")
                return
            # CSV形式:
            #  - ax, ay, az: 加速度センサーの各軸の値 (m/s^2)
            #  - gx, gy, gz: ジャイロスコープの各軸の値 (degree/sec)
            ax, ay, az, gx, gy, gz = map(float, parts[:6])
            
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            # センサのジャイロデータは degree/sec で出力されるので、
            # ラジアンに変換するための変換係数 (π/180) を適用
            conv = math.pi / 180  
            self.roll  += gx * conv * dt
            self.pitch += gy * conv * dt
            self.yaw   += gz * conv * dt
            
            # yawを 0〜2π の範囲に正規化（これによりログ上で負の値がなくなります）
            self.yaw = self.yaw % (2 * math.pi)
            
            
            quat = euler_to_quaternion(self.roll, self.pitch, self.yaw)
            self.publish_imu_data(quat, current_time)
            # info_once を info に置換
            self.get_logger().info(
                f"CSV: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}, roll={self.roll:.2f}, pitch={self.pitch:.2f}, yaw={self.yaw:.2f}"
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
