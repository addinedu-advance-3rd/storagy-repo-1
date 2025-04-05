import rclpy
import sys
import json
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MoveToGoal(Node):
    def __init__(self, location):
        super().__init__('move_to_goal')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ✅ 서버가 준비될 때까지 대기
        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("⏳ 네비게이션 서버 대기 중...")

        self.target_location = location  # 목표 위치 저장
        self.send_goal()  # 이동 시작

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0

        # ✅ 목표 위치 적용
        goal_msg.pose.pose.position.x = self.target_location['x']
        goal_msg.pose.pose.position.y = self.target_location['y']
        goal_msg.pose.pose.position.z = self.target_location['z']
        goal_msg.pose.pose.orientation.z = self.target_location['orientation_z']
        goal_msg.pose.pose.orientation.w = self.target_location['orientation_w']

        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 목표 거부됨!')
            return
        self.get_logger().info('✅ 목표 수락됨!')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('🎯 목표 완료!')
        else:
            self.get_logger().info('❌ 목표 실패!')

def main():
    rclpy.init()

    # ✅ `sys.argv[1]`으로 JSON 데이터 받기 (Escape 문자 문제 해결)
    if len(sys.argv) < 2:
        print("❌ 이동할 목표 좌표가 없습니다.")
        return
    
    # ✅ JSON 문자열을 안전하게 변환
    target_pose = json.loads(sys.argv[1])
    print(f"📌 목표 위치: {target_pose}")  # 디버깅용 출력

    node = MoveToGoal(target_pose)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
