import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations
import time

class Rotate90(Node):
    def __init__(self):
        super().__init__('rotate_90')
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 회전 상태 변수
        self.initial_yaw = None
        self.current_yaw = None
        self.target_reached = False

        # 회전 속도 (rad/s)
        self.angular_speed = 0.2
        
        # 타이머: 일정 주기로 회전 제어
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        # odometry 메시지에서 quaternion을 추출하고 yaw 계산
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # yaw
        
        # 초기 회전 시작 시점을 기록
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            self.get_logger().info(f"Initial yaw: {math.degrees(self.initial_yaw):.2f}°")

    def control_loop(self):
        if self.current_yaw is None or self.initial_yaw is None or self.target_reached:
            return

        # 현재 회전량 계산 (각도 차이, 라디안 단위)
        angle_turned = self.normalize_angle(self.current_yaw - self.initial_yaw)
        angle_turned_deg = math.degrees(abs(angle_turned))
        
        self.get_logger().info(f"Angle turned: {angle_turned_deg:.2f}°")
        
        twist = Twist()
        
        # 목표 90°에 도달할 때까지 회전 명령 전송
        if angle_turned_deg < 90.0:
            twist.angular.z = self.angular_speed if angle_turned >= 0 else -self.angular_speed
            twist.linear.x = 0.0
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.target_reached = True
            self.get_logger().info("90도 회전 완료!")
        
        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        """ angle을 [-pi, pi] 범위로 정규화 """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = Rotate90()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
