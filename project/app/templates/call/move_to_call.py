import rclpy
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class MoveToGoal(Node):
    def __init__(self, target_pose):
        super().__init__('move_to_goal')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        print("🔄 Action Server 대기 중...")
        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("⏳ 네비게이션 서버 대기 중...")

        print(f"📌 목표 위치 수신: {target_pose}")  # 🛠️ 입력된 좌표 확인
        x, y, z, orientation_z, orientation_w = map(float, target_pose.split(","))

        self.target_location = {
            "x": x, "y": y, "z": z,
            "orientation_z": orientation_z, "orientation_w": orientation_w
        }
        print(f"🎯 목표 위치: {self.target_location}")  # 🛠️ 변환된 목표 좌표 확인

        self.send_goal()  # 이동 시작

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0

        goal_msg.pose.pose.position.x = self.target_location["x"]
        goal_msg.pose.pose.position.y = self.target_location["y"]
        goal_msg.pose.pose.position.z = self.target_location["z"]
        goal_msg.pose.pose.orientation.z = self.target_location["orientation_z"]
        goal_msg.pose.pose.orientation.w = self.target_location["orientation_w"]

        print(f"📤 목표 전송: {goal_msg.pose.pose}")  # 🛠️ 실제로 전송되는 목표 좌표 확인

        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ 목표 거부됨!")
            return
        print("✅ 목표 수락됨! 🚀 이동 중...")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            print("🎯 목표 완료!")
        else:
            print("❌ 목표 실패!")

def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("❌ 이동할 목표 좌표가 없습니다.")
        return

    target_pose = sys.argv[1]
    print(f"📌 목표 위치: {target_pose}")  # 🛠️ 전달된 인자 확인

    node = MoveToGoal(target_pose)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
