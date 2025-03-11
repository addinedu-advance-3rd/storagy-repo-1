import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# send_goal.py → 목적지 좌표를 설정하고 목표를 전송
# arrival_listener.py → 목표 도착 여부를 감시하고 robot_arrival 토픽을 발행
# mission_handler.py → 로봇이 도착하면 해당 이벤트를 감지하고 추가 작업 수행

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')
        self.arrival_received = False  
        self.subscription = self.create_subscription(
            String,
            'robot_arrival',  
            self.handle_arrival,
            10
        )

    def handle_arrival(self, msg):
        """ 로봇 도착 이벤트 처리 """
        self.get_logger().info(f"📦 로봇 도착 이벤트 수신: {msg.data}")
        self.arrival_received = True  

    def wait_for_arrival(self):
        """ 로봇이 도착할 때까지 대기 """
        self.get_logger().info("📦 로봇이 도착할 때까지 기다리고 있습니다.")
        while not self.arrival_received:
            self.get_logger().info("⏳ 대기 중...")  
            rclpy.spin_once(self, timeout_sec=1.0)  
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MissionHandler()
    try:
        node.wait_for_arrival()  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
