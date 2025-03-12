import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool  # Bool 메시지 타입 추가
import math
import tf_transformations
from std_srvs.srv import Trigger

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
        
        # 회전 완료 토픽 발행자 추가 - 네임스페이스 확인
        self.rotation_done_pub = self.create_publisher(Bool, '/rotation_done', 10)
        self.get_logger().info("회전 완료 토픽 발행자 생성: /rotation_done")
        
        # 회전 서비스 추가
        self.rotate_service = self.create_service(
            Trigger,  # std_srvs.srv.Trigger
            'rotate_90_degrees',
            self.rotate_service_callback
        )
        
        # 회전 상태 변수
        self.initial_yaw = None
        self.current_yaw = None
        self.target_reached = False
        self.rotating = False
        self.counter = 0
        self.publish_count = 0  # 메시지 발행 카운터 추가
        
        # 회전 속도 (rad/s)
        self.angular_speed = 0.2  
        
        # 타이머: 일정 주기로 회전 제어
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 완료 메시지 발행 타이머 추가
        self.done_timer = None

    def rotate_service_callback(self, request, response):
        """90도 회전 서비스 콜백"""
        if self.rotating:
            response.success = False
            response.message = "이미 회전 중입니다."
            return response
            
        # 회전 초기화
        self.initial_yaw = None
        self.target_reached = False
        self.rotating = True
        self.counter = 0
        self.publish_count = 0
        
        # 회전 시작 시 False 상태 발행
        done_msg = Bool()
        done_msg.data = False
        self.rotation_done_pub.publish(done_msg)
        
        # 서비스 응답
        response.success = True
        response.message = "90도 회전을 시작합니다."
        return response

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
        if self.rotating and self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            self.get_logger().info(f"Initial yaw: {math.degrees(self.initial_yaw):.2f}°")

    def publish_done_message(self):
        """회전 완료 메시지 발행 (타이머 콜백)"""
        if self.publish_count < 5:
            done_msg = Bool()
            done_msg.data = True
            self.rotation_done_pub.publish(done_msg)
            self.get_logger().info(f"회전 완료 메시지 발행! ({self.publish_count+1}/5)")
            self.publish_count += 1
        else:
            # 5번 발행 완료 후 타이머 취소
            if self.done_timer:
                self.done_timer.cancel()
                self.done_timer = None
            self.rotating = False

    def control_loop(self):
        if self.counter < 5:
            if not self.rotating or self.current_yaw is None or self.initial_yaw is None:
                return
                
            if self.target_reached:
                # 목표에 도달했으면 정지 상태 유지
                twist = Twist()
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                
                # 회전 완료 메시지 발행 타이머 설정 (time.sleep 대신 타이머 사용)
                if self.done_timer is None:
                    self.done_timer = self.create_timer(0.1, self.publish_done_message)
                
                self.counter += 1
                return

            # 현재 회전량 계산 (각도 차이, 라디안 단위)
            angle_turned = self.normalize_angle(self.current_yaw - self.initial_yaw)
            angle_turned_deg = math.degrees(abs(angle_turned))
            
            self.get_logger().info(f"Angle turned: {angle_turned_deg:.2f}°")
            
            twist = Twist()
            
            # 목표 90°에 도달할 때까지 회전 명령 전송
            if angle_turned_deg < 89.0:  # 약간 여유를 두어 89도로 설정
                twist.angular.z = self.angular_speed
                twist.linear.x = 0.0
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.target_reached = True
                self.get_logger().info("90도 회전 완료!")
                
                # 회전 완료 메시지 발행
                done_msg = Bool()
                done_msg.data = True
                self.rotation_done_pub.publish(done_msg)
            
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().info("90도 회전 cmd 발행 중단!")
            return

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
