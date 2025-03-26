import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker

class RvizNode(Node):
    def __init__(self):
        super().__init__('rviz_node')
        self.pose_subscription = self.create_subscription(PoseStamped, '/imu_pose', self.pose_callback, 10)
        self.twist_subscription = self.create_subscription(TwistStamped, '/imu_twist', self.twist_callback, 10)
        self.pose_marker_publisher = self.create_publisher(Marker, 'imu_pose_marker', 10)
        self.twist_marker_publisher = self.create_publisher(Marker, 'imu_twist_marker', 10)

    def pose_callback(self, msg: PoseStamped):
        # デバッグ用ログ: Poseデータを確認
        self.get_logger().info(f"Pose received: x={msg.pose.position.x:.2f} mm, y={msg.pose.position.y:.2f} mm, z={msg.pose.position.z:.2f} mm")

        # Poseデータを可視化
        marker = Marker()
        marker.header = msg.header
        marker.ns = "pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 0.1  # スケールを小さく調整
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.pose_marker_publisher.publish(marker)

    def twist_callback(self, msg: TwistStamped):
        # デバッグ用ログ: Twistデータを確認
        self.get_logger().info(f"Twist received: linear_x={msg.twist.linear.x:.2f} mm/s, linear_y={msg.twist.linear.y:.2f} mm/s, linear_z={msg.twist.linear.z:.2f} mm/s")

        # Twistデータを可視化
        marker = Marker()
        marker.header = msg.header
        marker.ns = "twist"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = msg.twist.linear.x / 1000.0  # [m]
        marker.pose.position.y = msg.twist.linear.y / 1000.0  # [m]
        marker.pose.position.z = msg.twist.linear.z / 1000.0  # [m]
        marker.scale.x = 0.1  # スケールを小さく調整
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.twist_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RvizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
