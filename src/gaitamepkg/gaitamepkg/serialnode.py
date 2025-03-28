import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, TransformStamped, TwistStamped
import tf2_ros
import serial
import struct

def hex_to_float(hex_str):
    """
    16進数文字列をIEEE 754形式の浮動小数点数に変換する関数
    """
    int_val = int(hex_str, 16)
    packed = struct.pack('I', int_val)
    return struct.unpack('f', packed)[0]

class SerialRosNode(Node):
    def __init__(self):
        super().__init__('serial_ros_node')

        # パラメータの宣言と取得
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('tf_parent_frame', 'world')
        self.declare_parameter('tf_child_frame', 'imu_link')

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.tf_parent_frame = self.get_parameter('tf_parent_frame').get_parameter_value().string_value
        self.tf_child_frame = self.get_parameter('tf_child_frame').get_parameter_value().string_value

        self.get_logger().info(f"Using serial port: {serial_port} at {baud_rate} baud")
        self.get_logger().info(f"TF frames: parent='{self.tf_parent_frame}', child='{self.tf_child_frame}'")

        # パブリッシャの作成
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pose_pub = self.create_publisher(Pose, '/imu_pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/imu_twist', 10)  # TwistStamped用パブリッシャを追加

        # tf2ブロードキャスターの作成
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # シリアルポートの初期化
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Serial port {serial_port} opened at {baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

        # タイマー：定期的にシリアルからデータ読み込み（例:10ms間隔）
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is None:
            return

        try:
            # 1行分読み込み（改行まで）
            raw_line = self.ser.readline()
            try:
                line = raw_line.decode('utf-8', errors='ignore').strip()
            except UnicodeDecodeError as e:
                self.get_logger().error(f"Unicode decode error: {e}")
                return

            if not line:
                return

            # デバッグ用ログ: 生データを表示
            self.get_logger().debug(f"Raw data: {raw_line}")
            self.get_logger().info(f"Decoded data: {line}")

            # カンマ区切りで分割
            parts = line.split(',')
            if len(parts) != 18:
                self.get_logger().warn(f"Unexpected data length: {parts}")
                return

            # 各値の変換
            try:
                timestamp = int(parts[0], 16)
                temperature = hex_to_float(parts[1])
                angular_velocity = [hex_to_float(p) for p in parts[2:5]]
                linear_acceleration = [hex_to_float(p) for p in parts[5:8]]
                quat_vals = [hex_to_float(p) for p in parts[8:12]]
                velocity = [hex_to_float(p) for p in parts[12:15]]
                position = [hex_to_float(p) for p in parts[15:18]]
            except ValueError as e:
                self.get_logger().error(f"Error parsing data: {line}, {e}")
                return

            # ROSの現在時刻をヘッダーにセット
            now = self.get_clock().now().to_msg()

            # IMUメッセージの作成
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.tf_child_frame
            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]
            imu_msg.linear_acceleration.x = linear_acceleration[0]
            imu_msg.linear_acceleration.y = linear_acceleration[1]
            imu_msg.linear_acceleration.z = linear_acceleration[2]
            imu_msg.orientation.w = quat_vals[0]
            imu_msg.orientation.x = quat_vals[1]
            imu_msg.orientation.y = quat_vals[2]
            imu_msg.orientation.z = quat_vals[3]
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1

            # Poseメッセージの作成
            pose_msg = Pose()
            pose_msg.position.x = position[0]
            pose_msg.position.y = position[1]
            pose_msg.position.z = position[2]
            pose_msg.orientation.w = quat_vals[0]
            pose_msg.orientation.x = quat_vals[1]
            pose_msg.orientation.y = quat_vals[2]
            pose_msg.orientation.z = quat_vals[3]

            # TwistStampedメッセージの作成
            twist_msg = TwistStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = self.tf_child_frame
            twist_msg.twist.linear.x = velocity[0] * 1000.0  # [mm/s]
            twist_msg.twist.linear.y = velocity[1] * 1000.0  # [mm/s]
            twist_msg.twist.linear.z = velocity[2] * 1000.0  # [mm/s]
            twist_msg.twist.angular.x = angular_velocity[0]  # [rad/s]
            twist_msg.twist.angular.y = angular_velocity[1]  # [rad/s]
            twist_msg.twist.angular.z = angular_velocity[2]  # [rad/s]

            # パブリッシュ
            self.imu_pub.publish(imu_msg)
            self.pose_pub.publish(pose_msg)
            self.twist_pub.publish(twist_msg)  # TwistStampedをパブリッシュ

            # TransformStampedメッセージを作成し、/tfにパブリッシュ
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.tf_parent_frame
            t.child_frame_id = self.tf_child_frame
            t.transform.translation.x = pose_msg.position.x
            t.transform.translation.y = pose_msg.position.y
            t.transform.translation.z = pose_msg.position.z
            t.transform.rotation.w = pose_msg.orientation.w
            t.transform.rotation.x = pose_msg.orientation.x
            t.transform.rotation.y = pose_msg.orientation.y
            t.transform.rotation.z = pose_msg.orientation.z

            self.tf_broadcaster.sendTransform(t)

            # デバッグ用ログ
            self.get_logger().debug(f"Published IMU and Pose (timestamp: {timestamp})")
            self.get_logger().debug(f"Published Twist: linear=({twist_msg.twist.linear.x:.2f}, {twist_msg.twist.linear.y:.2f}, {twist_msg.twist.linear.z:.2f}), "
                                     f"angular=({twist_msg.twist.angular.x:.2f}, {twist_msg.twist.angular.y:.2f}, {twist_msg.twist.angular.z:.2f})")

        except Exception as e:
            self.get_logger().error(f"Error processing serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
