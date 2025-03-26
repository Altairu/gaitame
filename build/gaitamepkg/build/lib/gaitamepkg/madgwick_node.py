import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from std_msgs.msg import String  # 新規追加
import serial  # 追加: シリアル通信用ライブラリ

# A simple implementation of the Madgwick filter
class MadgwickAHRS:
    def __init__(self, beta=0.1):
        self.beta = beta
        # Quaternion stored as [w, x, y, z]
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    
    def update(self, gyroscope, accelerometer, dt, magnetometer=None):
        # Using a simplified update that ignores magnetometer data
        q = self.quaternion
        gx, gy, gz = gyroscope    # in rad/s
        ax, ay, az = accelerometer

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax /= norm; ay /= norm; az /= norm

        # Auxiliary variables
        _2q0 = 2 * q[0]
        _2q1 = 2 * q[1]
        _2q2 = 2 * q[2]
        _2q3 = 2 * q[3]
        q0q0 = q[0] * q[0]
        q1q1 = q[1] * q[1]
        q2q2 = q[2] * q[2]
        q3q3 = q[3] * q[3]

        # Gradient descent corrective step (simplified)
        s0 = _2q0 * q2q2 + _2q0 * q1q1 - _2q1 * ay - _2q2 * ax
        s1 = _2q1 * q3q3 + _2q0 * ay - _2q1 * q0q0 - _2q2 * az
        s2 = _2q2 * q0q0 + _2q0 * ax - _2q3 * ay - _2q2 * q3q3
        s3 = _2q3 * q1q1 - _2q1 * ax + _2q3 * q0q0 - _2q2 * ay
        norm_s = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm_s == 0:
            return
        s0 /= norm_s; s1 /= norm_s; s2 /= norm_s; s3 /= norm_s

        # Compute rate of change of quaternion using dt parameter here instead of fixed sample_period
        q_dot0 = 0.5 * (-q[1]*gx - q[2]*gy - q[3]*gz) - self.beta * s0
        q_dot1 = 0.5 * ( q[0]*gx + q[2]*gz - q[3]*gy) - self.beta * s1
        q_dot2 = 0.5 * ( q[0]*gy - q[1]*gz + q[3]*gx) - self.beta * s2
        q_dot3 = 0.5 * ( q[0]*gz + q[1]*gy - q[2]*gx) - self.beta * s3

        q += np.array([q_dot0, q_dot1, q_dot2, q_dot3]) * dt
        q /= np.linalg.norm(q)
        self.quaternion = q
    
    def get_quaternion(self):
        return self.quaternion.copy()

class MadgwickNode(Node):
    def __init__(self):
        super().__init__('madgwick_node')
        # 既存のIMUデータサブスクリプション
        self.subscription = self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        # CSV形式のデータサブスクリプションを追加
        self.csv_subscription = self.create_subscription(String, 'imu/csv', self.csv_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, 'imu_pose_madgwick', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.filter = MadgwickAHRS(beta=0.1)
        # Initialize last_time for dt calculation
        self.last_time = self.get_clock().now()
        # 追加: シリアル通信の初期化
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info(f"Opened serial port: {self.ser.name}")
            self.use_serial = True
            self.create_timer(0.005, self.serial_timer_callback)
        except serial.SerialException as e:
            self.get_logger().warning(f"Serial port not opened: {e}")
            self.use_serial = False

    def imu_callback(self, msg: Imu):
        # Compute dt using current time and self.last_time
        current_time = msg.header.stamp  # Assuming msg.header.stamp is valid
        # Fallback: use node’s clock if header stamp unavailable
        if current_time.sec == 0 and current_time.nanosec == 0:
            current_time = self.get_clock().now().to_msg()
        # dt in seconds: (using node clock for simplicity)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        self.filter.update((gx, gy, gz), (ax, ay, az), dt)
        q = self.filter.get_quaternion()  # q is [w, x, y, z]
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "imu_link"
        # Convert to PoseStamped convention (x, y, z, w)
        pose_msg.pose.orientation.x = q[1]
        pose_msg.pose.orientation.y = q[2]
        pose_msg.pose.orientation.z = q[3]
        pose_msg.pose.orientation.w = q[0]
        self.publisher_.publish(pose_msg)
        
        # Publish TF
        t_msg = TransformStamped()
        t_msg.header.stamp = msg.header.stamp
        t_msg.header.frame_id = "base_link"
        t_msg.child_frame_id = "imu_link"
        t_msg.transform.translation.x = 0.0
        t_msg.transform.translation.y = 0.0
        t_msg.transform.translation.z = 0.0
        t_msg.transform.rotation.x = q[1]
        t_msg.transform.rotation.y = q[2]
        t_msg.transform.rotation.z = q[3]
        t_msg.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t_msg)
        self.get_logger().info(
            f"Madgwick Orientation: roll={math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]))):.2f}°, " +
            f"pitch={math.degrees(math.asin(2*(q[0]*q[2]-q[3]*q[1]))):.2f}°, " +
            f"yaw={(math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3])))+360)%360:.2f}°"
        )

    def csv_callback(self, msg: String):
        try:
            data = msg.data.strip().split(',')
            if len(data) != 6:
                self.get_logger().warn("CSV data does not have 6 elements")
                return
            ax, ay, az, gx, gy, gz = map(float, data)
            # ジャイロ値は degree/secの場合、ラジアン/secに変換
            gx = gx * (math.pi / 180)
            gy = gy * (math.pi / 180)
            gz = gz * (math.pi / 180)
            # Compute dt using node clock
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            self.filter.update((gx, gy, gz), (ax, ay, az), dt)
            q = self.filter.get_quaternion()  # q: [w, x, y, z]
            
            pose_msg = PoseStamped()
            # 現在時刻を利用（文字列変換は任意）
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "imu_link"
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]
            pose_msg.pose.orientation.w = q[0]
            self.publisher_.publish(pose_msg)
            
            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = "base_link"
            t_msg.child_frame_id = "imu_link"
            t_msg.transform.translation.x = 0.0
            t_msg.transform.translation.y = 0.0
            t_msg.transform.translation.z = 0.0
            t_msg.transform.rotation.x = q[1]
            t_msg.transform.rotation.y = q[2]
            t_msg.transform.rotation.z = q[3]
            t_msg.transform.rotation.w = q[0]
            self.tf_broadcaster.sendTransform(t_msg)
            self.get_logger().info("CSV Orientation processed")
        except Exception as e:
            self.get_logger().error("CSV parse error: " + str(e))

    def serial_timer_callback(self):
        try:
            # 追加: シリアルバッファにデータがなければログ出力して抜ける
            if self.ser.in_waiting == 0:
                self.get_logger().debug("No data available in serial buffer.")
                return
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or "ax" in line:
                return
            parts = line.split(',')
            if len(parts) != 6:
                self.get_logger().warn(f"Serial CSV does not have 6 fields: {line}")
                return
            ax, ay, az, gx, gy, gz = map(float, parts)
            # ジャイロ値を degree/sec から rad/sec に変換
            gx *= (math.pi / 180)
            gy *= (math.pi / 180)
            gz *= (math.pi / 180)
            # Compute dt using node clock
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            self.filter.update((gx, gy, gz), (ax, ay, az), dt)
            q = self.filter.get_quaternion()
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "imu_link"
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]
            pose_msg.pose.orientation.w = q[0]
            self.publisher_.publish(pose_msg)
            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = "base_link"
            t_msg.child_frame_id = "imu_link"
            t_msg.transform.translation.x = 0.0
            t_msg.transform.translation.y = 0.0
            t_msg.transform.translation.z = 0.0
            t_msg.transform.rotation.x = q[1]
            t_msg.transform.rotation.y = q[2]
            t_msg.transform.rotation.z = q[3]
            t_msg.transform.rotation.w = q[0]
            self.tf_broadcaster.sendTransform(t_msg)
            self.get_logger().info("Serial Orientation processed")
        except Exception as e:
            self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MadgwickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
