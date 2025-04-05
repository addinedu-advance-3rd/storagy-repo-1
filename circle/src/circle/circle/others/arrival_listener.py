import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class ArrivalListener(Node):
    def __init__(self):
        super().__init__('arrival_listener')

        # 📌 도착 이벤트를 발행하는 토픽 생성
        self.arrival_publisher = self.create_publisher(String, '/robot_arrival', 10)

        # 📌 Storagy 네비게이션 액션 클라이언트 생성
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ✅ goal_msg를 여기서 미리 정의하면 안 됨!
        self.goal_future = None  # ✅ 초기에 None으로 설정

        # 📌 새로운 목표 도착을 감지하기 위한 타이머 실행
        self.timer = self.create_timer(0.5, self.check_nav_status)  # ✅ 함수가 존재하도록 수정

        # 📌 액션 서버 연결
        self.get_logger().info("📡 Storagy 네비게이션 감시 시작!")
        self.action_client.wait_for_server()

    def check_nav_status(self):
        """ 현재 네비게이션 목표의 상태를 체크하는 메서드 """
        if self.goal_future is None:
            return

        goal_handle = self.goal_future.result()
        if goal_handle is None:
            return

        # ✅ 목표 도착 성공 여부 확인
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎯 네비게이션 성공! `/robot_arrival` 발행")
            msg = String()
            msg.data = "arrived"
            self.arrival_publisher.publish(msg)

            # ✅ 새로운 목표를 계속 감시할 수 있도록 초기화
            self.goal_future = None
        elif goal_handle.status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            self.get_logger().error("❌ 네비게이션 실패!")
            self.goal_future = None  # 실패 시에도 초기화

    def send_goal(self, goal_pose):
        """ 네비게이션 목표를 전송하고 상태 감시 """
        goal_msg = NavigateToPose.Goal()  # ✅ `goal_msg` 정의
        goal_msg.pose = goal_pose

        self.get_logger().info(f"🚀 목표 전송: {goal_pose}")

        self.goal_future = self.action_client.send_goal_async(goal_msg)  # ✅ `goal_msg` 정의 후 사용
        if self.goal_future is not None:
            self.goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error("❌ send_goal_async()가 None을 반환함. 액션 서버가 실행 중인지 확인!")

    def goal_response_callback(self, future):
        """ 네비게이션 목표가 성공적으로 수락되었는지 확인 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ 목표가 거부됨!")
            return

        self.get_logger().info("✅ 목표가 수락됨!")
        self.goal_future = goal_handle.get_result_async()
        self.goal_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ 네비게이션 결과를 확인하고 도착 이벤트 발행 """
        result = future.result()
        print(result)
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎯 네비게이션 성공! `/robot_arrival` 발행")
            msg = String()
            msg.data = "arrived"
            self.arrival_publisher.publish(msg)
        else:
            self.get_logger().error("❌ 네비게이션 실패!")

        # ✅ 새로운 목표를 계속 감시할 수 있도록 초기화
        self.goal_future = None

def main(args=None):
    rclpy.init(args=args)
    node = ArrivalListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
