import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math

def quaternion_to_euler(x, y, z, w):
    # オイラー角 (ラジアン) の計算
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # 度に変換
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    # yaw角を 0～360 度に正規化
    if yaw_deg < 0:
        yaw_deg += 360
    return roll_deg, pitch_deg, yaw_deg

class RvizNode(Node):
    def __init__(self):
        super().__init__('rviz_node')
        self.subscription = self.create_subscription(PoseStamped, 'imu_pose', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Marker, 'imu_marker', 10)

    def listener_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received PoseStamped: {msg}")
        # 受信したPoseStampedは、マイコン側で計算された姿勢データをもとに生成されています。
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(x, y, z, w)
        
        marker = Marker()
        marker.header = msg.header
        marker.ns = "imu"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.text = f"Roll: {roll:.1f}\nPitch: {pitch:.1f}\nYaw: {yaw:.1f}"
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.publisher_.publish(marker)
        self.get_logger().info(f"Published Marker: {marker}")

def main(args=None):
    rclpy.init(args=args)
    node = RvizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
