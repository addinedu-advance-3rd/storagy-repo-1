import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger  # ✅ Trigger 서비스 가져오기

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.client.wait_for_server()
        self.arrival_publisher = self.create_publisher(String, '/robot_arrival', 10)
        
        # ✅ ArUco 도킹 서비스 클라이언트 생성 - 비동기적으로 처리
        self.dock_client = self.create_client(Trigger, '/dock_robot')
        # 도킹 서비스 연결 확인을 위한 타이머 생성
        self.dock_service_timer = self.create_timer(1.0, self.check_dock_service)
        self.dock_service_ready = False
        
        # 이동 목표 설정 - 도킹 서비스 준비와 상관없이 바로 시작
        self.send_goal()

        # 이동 중 ArUco 마커를 감지하는 타이머 실행
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.docking_mode = False

    def send_goal(self):
        """ 네비게이션 목표 좌표 설정 및 전송 """
        self.get_logger().info('🛤️ 목표 지점으로 이동 시작...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  
        goal_msg.pose.header.stamp.sec = 0  
        goal_msg.pose.header.stamp.nanosec = 0  

        # 목표 좌표 설정
        goal_msg.pose.pose.position.x = -2.074
        goal_msg.pose.pose.position.y = 0.270
        goal_msg.pose.pose.position.z = 0.065  

        # 방향 (Quaternion) 설정
        goal_msg.pose.pose.orientation.z = -0.013
        goal_msg.pose.pose.orientation.w = 1.000

        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        """ 네비게이션 서버가 목표를 수락했는지 확인 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 목표 거부됨!')
            return
        
        self.get_logger().info('✅ 목표 수락됨!')
        
        # 목표가 완료될 때까지 기다림
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        """ 목표 도착 후 도킹 시도 """
        if self.docking_mode:  # 이미 도킹 모드로 전환한 경우 무시
            return

        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('🎯 목표 완료! ArUco 도킹 시도...')
            self.call_docking_service()
        else:
            self.get_logger().info('❌ 목표 도착 실패!')

    def timer_callback(self):
        """ 이동 중 ArUco 마커를 감지하면 네비게이션을 취소하고 도킹 모드로 전환 """
        if self.docking_mode:  # 이미 도킹 모드라면 실행하지 않음
            return

        if not self.client.server_is_ready():  # 네비게이션이 실행 중인지 확인
            return

        self.get_logger().info("✅ 이동 진행 중... (ArUco 마커 확인 중)")

    def check_dock_service(self):
        """도킹 서비스 가용성을 주기적으로 확인"""
        if self.dock_service_ready:
            # 이미 서비스가 준비되었으면 타이머 취소
            self.dock_service_timer.cancel()
            return
            
        if self.dock_client.service_is_ready():
            self.get_logger().info('도킹 서비스 연결 완료')
            self.dock_service_ready = True
            # 서비스가 준비되면 타이머 취소
            self.dock_service_timer.cancel()

    def call_docking_service(self):
        """ ArUco 마커를 기반으로 도킹 서비스 호출 """

        if self.docking_mode:
            self.get_logger().info('🎯 이미 도킹 모드로 전환됨!')
            return

        if not self.dock_service_ready:
            self.get_logger().error('❌ 도킹 서비스가 아직 준비되지 않았습니다!')
            return

        self.docking_mode = True
        self.get_logger().info('🎯 도킹 모드로 전환됨!')
        
        self.get_logger().info('📡 도킹 서비스 호출 중...')
        try:
            request = Trigger.Request()
            future = self.dock_client.call_async(request)
            future.add_done_callback(self.docking_response_callback)
        except Exception as e:
            self.get_logger().error(f'🚨 도킹 서비스 요청 실패: {e}')
            self.docking_mode = False
            self.get_logger().info('🎯 도킹 모드 해제됨!')

    def docking_response_callback(self, future: Future):
        """ 도킹 서비스 응답 처리 """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 도킹 성공: {response.message}')
            else:
                self.get_logger().info(f'❌ 도킹 실패: {response.message}')
                self.docking_mode = False
                self.get_logger().info('🎯 도킹 모드 해제됨!')
        except Exception as e:
            self.get_logger().error(f'🚨 도킹 서비스 요청 실패: {e}')
            self.docking_mode = False
            self.get_logger().info('🎯 도킹 모드 해제됨!')

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
