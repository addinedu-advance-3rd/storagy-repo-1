import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        
        # 네비게이션 액션 클라이언트
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # 로봇 직접 제어를 위한 cmd_vel 발행자 추가
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 도착 알림 발행자
        self.arrival_publisher = self.create_publisher(String, '/robot_arrival', 10)
        
        # 도킹 서비스 클라이언트
        self.get_logger().info("도킹 서비스 클라이언트 생성: /dock_robot")
        self.dock_client = self.create_client(Trigger, '/dock_robot')
        
        # 상태 변수
        self.dock_service_ready = False
        self.docking_mode = False
        self.navigation_started = False
        
        # 타이머 저장 변수
        self.delayed_timers = []
        
        # 서비스 연결 확인 타이머 (더 짧은 간격으로 확인)
        self.dock_service_timer = self.create_timer(0.5, self.check_dock_service)
        
        # 네비게이션 서버 연결 확인 타이머
        self.nav_server_timer = self.create_timer(1.0, self.check_navigation_server)
        
        # 상태 모니터링 타이머
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("목표 지점 이동 노드 초기화 완료")

    def create_delayed_timer(self, delay, callback):
        """일회성 지연 타이머 생성 (oneshot 대체)"""
        timer = self.create_timer(delay, lambda: self.delayed_callback(timer, callback))
        self.delayed_timers.append(timer)
        return timer
        
    def delayed_callback(self, timer, callback):
        """지연 타이머 콜백 처리"""
        # 타이머 취소
        timer.cancel()
        if timer in self.delayed_timers:
            self.delayed_timers.remove(timer)
        # 원래 콜백 실행
        callback()

    def check_navigation_server(self):
        """네비게이션 서버 연결 확인"""
        if self.navigation_started:
            self.nav_server_timer.cancel()
            return
            
        if self.client.server_is_ready():
            self.get_logger().info("네비게이션 서버 연결 완료!")
            self.nav_server_timer.cancel()
            # 네비게이션 시작
            self.send_goal()
            self.navigation_started = True
        else:
            self.get_logger().info("네비게이션 서버 연결 대기 중...")
            # 서버 연결이 안 되면 로봇을 제자리에서 회전시켜 활성화
            self.rotate_in_place()

    def rotate_in_place(self):
        """네비게이션 서버 활성화를 위해 로봇을 제자리에서 회전"""
        twist = Twist()
        twist.angular.z = 0.1  # 느린 속도로 회전
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("로봇 제자리 회전 중 (네비게이션 서버 활성화 시도)")

    def send_goal(self):
        """네비게이션 목표 좌표 설정 및 전송"""
        self.get_logger().info('🛤️ 목표 지점으로 이동 시작...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  
        goal_msg.pose.header.stamp.sec = 0  
        goal_msg.pose.header.stamp.nanosec = 0  

        # 목표 좌표 설정
        goal_msg.pose.pose.position.x = -0.284
        goal_msg.pose.pose.position.y = 0.137
        goal_msg.pose.pose.position.z = 0.0  

        # 방향 (Quaternion) 설정
        goal_msg.pose.pose.orientation.z = -0.733
        goal_msg.pose.pose.orientation.w = 0.679

        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        """네비게이션 서버가 목표를 수락했는지 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 목표 거부됨! 다시 시도합니다.')
            # 목표가 거부되면 잠시 후 다시 시도
            self.create_delayed_timer(2.0, self.send_goal)
            return
        
        self.get_logger().info('✅ 목표 수락됨! 이동 중...')
        
        # 목표가 완료될 때까지 기다림
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        """목표 도착 후 도킹 시도"""
        if self.docking_mode:  # 이미 도킹 모드로 전환한 경우 무시
            return

        result = future.result()
        status_str = {
            1: "STATUS_UNKNOWN",
            2: "STATUS_ACCEPTED", 
            3: "STATUS_CANCELING",
            4: "STATUS_SUCCEEDED",
            5: "STATUS_CANCELED",
            6: "STATUS_ABORTED"
        }.get(result.status, f"UNKNOWN({result.status})")
        
        self.get_logger().info(f'네비게이션 결과: {status_str}')
        
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('🎯 목표 완료! ArUco 도킹 시도...')
            # 도킹 서비스 호출 전 잠시 대기 (안정화를 위해)
            self.create_delayed_timer(1.0, self.call_docking_service)
        else:
            self.get_logger().info(f'❌ 목표 도착 실패! 상태: {status_str}')
            # 실패 시 다시 시도
            self.create_delayed_timer(3.0, self.send_goal)

    def timer_callback(self):
        """상태 모니터링 및 필요시 조치"""
        if self.docking_mode:
            # 도킹 모드에서는 모니터링만 수행
            self.get_logger().info("🔄 도킹 모드 진행 중...")
            return

        if self.navigation_started and not self.client.server_is_ready():
            # 네비게이션 서버 연결이 끊어진 경우
            self.get_logger().warn("⚠️ 네비게이션 서버 연결 끊김! 재연결 시도...")
            self.navigation_started = False
            self.nav_server_timer = self.create_timer(1.0, self.check_navigation_server)
            return

        if self.navigation_started:
            self.get_logger().info("✅ 이동 진행 중...")

    def check_dock_service(self):
        """도킹 서비스 가용성을 주기적으로 확인"""
        if self.dock_service_ready:
            self.dock_service_timer.cancel()
            return
            
        self.get_logger().info("도킹 서비스 확인 중...")
        
        if self.dock_client.service_is_ready():
            self.get_logger().info('✅ 도킹 서비스 연결 완료!')
            self.dock_service_ready = True
            self.dock_service_timer.cancel()
        else:
            self.get_logger().info('⏳ 도킹 서비스 아직 준비되지 않음')

    def call_docking_service(self):
        """ArUco 마커를 기반으로 도킹 서비스 호출"""
        if self.docking_mode:
            self.get_logger().info('🔄 이미 도킹 모드로 전환됨!')
            return

        if not self.dock_service_ready:
            self.get_logger().error('❌ 도킹 서비스가 아직 준비되지 않았습니다!')
            # 서비스가 준비되지 않았으면 잠시 후 다시 확인
            self.create_delayed_timer(2.0, self.call_docking_service)
            return

        self.docking_mode = True
        self.get_logger().info('🎯 도킹 모드로 전환됨!')
        
        self.get_logger().info('📡 도킹 서비스 호출 중...')
        try:
            request = Trigger.Request()
            self.get_logger().info('📡 도킹 서비스 요청 객체 생성 완료')
            future = self.dock_client.call_async(request)
            self.get_logger().info('📡 도킹 서비스 비동기 호출 완료, 응답 대기 중...')
            future.add_done_callback(self.docking_response_callback)
        except Exception as e:
            self.get_logger().error(f'🚨 도킹 서비스 요청 실패: {e}')
            self.docking_mode = False
            self.get_logger().info('🔄 도킹 모드 해제됨! 3초 후 재시도...')
            # 실패 시 3초 후 재시도
            self.create_delayed_timer(3.0, self.call_docking_service)

    def docking_response_callback(self, future: Future):
        """도킹 서비스 응답 처리"""
        try:
            self.get_logger().info('📡 도킹 서비스 응답 수신!')
            response = future.result()
            self.get_logger().info(f'📡 도킹 서비스 응답 내용: success={response.success}, message={response.message}')
            
            if response.success:
                self.get_logger().info(f'✅ 도킹 성공: {response.message}')
                # 도킹 성공 메시지 발행
                msg = String()
                msg.data = "DOCKING_COMPLETE"
                self.arrival_publisher.publish(msg)
            else:
                self.get_logger().info(f'❌ 도킹 실패: {response.message}')
                self.docking_mode = False
                self.get_logger().info('🔄 도킹 모드 해제됨! 5초 후 재시도...')
                # 실패 시 5초 후 재시도
                self.create_delayed_timer(5.0, self.call_docking_service)
        except Exception as e:
            self.get_logger().error(f'🚨 도킹 서비스 응답 처리 실패: {e}')
            self.docking_mode = False
            self.get_logger().info('🔄 도킹 모드 해제됨! 3초 후 재시도...')
            # 실패 시 3초 후 재시도
            self.create_delayed_timer(3.0, self.call_docking_service)

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
