import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

# 저장할 폴더 생성
save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.image_count = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 현재 프레임을 저장 (이름: calib_0.jpg, calib_1.jpg ...)
        filename = os.path.join(save_dir, f'calib_{self.image_count}.jpg')
        cv2.imwrite(filename, frame)
        self.image_count += 1

        self.get_logger().info(f"이미지 저장 완료: {filename}")
        time.sleep(0.2)

        # 캘리브레이션용 20장 정도 저장하면 종료
        if self.image_count >= 20:
            self.get_logger().info("캘리브레이션 이미지 저장 완료. 종료합니다.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCapture()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
