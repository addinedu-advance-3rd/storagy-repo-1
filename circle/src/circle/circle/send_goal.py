import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from rclpy.action.client import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.client.wait_for_server()
        self.arrival_publisher = self.create_publisher(String, '/robot_arrival', 10)
        self.timer = None
        self.send_goal()

        # 📌 도착 이벤트를 발행하는 토픽 생성

        self.location_A = {'x': 0.022, 'y': -0.558, 'z': 0.065, 'orientation_z' : -0.215, 'orientation_w': 0.977}
        self.location_B = {'x': 0.022, 'y': -0.558, 'z': 0.065, 'orientation_z' : -0.215, 'orientation_w': 0.977}
        self.location_home = {'x': -2.345, 'y': -0.095, 'z': 0.065, 'orientation_z' : 0.851, 'orientation_w': 0.525}

    # def send_goal(self, location):
    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  
        goal_msg.pose.header.stamp.sec = 0  
        goal_msg.pose.header.stamp.nanosec = 0  

        # 목표 좌표 설정
        goal_msg.pose.pose.position.x = -2.720
        goal_msg.pose.pose.position.y = 0.156
        goal_msg.pose.pose.position.z = 0.065  

        # 방향 (Quaternion) 설정
        goal_msg.pose.pose.orientation.z = 0.868
        goal_msg.pose.pose.orientation.w = 0.496

        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        print("✅ 목표 이동 진행중")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 목표 거부됨!')
            self.timer.cancel()
            return
        self.get_logger().info('✅ 목표 수락됨!')
        self.timer.cancel()

        # 목표가 완료될 때까지 기다림
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('🎯 목표 완료!')
            self.timer.cancel()
            msg = String()
            msg.data = "arrived"
            self.arrival_publisher.publish(msg)
        else:
            self.get_logger().info('❌ 목표 실패!')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
