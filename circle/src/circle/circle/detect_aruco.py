import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import os
# 카메라 캘리브레이션 데이터 로드
# calib_data = np.load("circle/src/circle/vision/calib_data.npz")  # 체커보드 캘리브레이션 데이터
calib_data = np.load("calib_data.npz")  # 체커보드 캘리브레이션 데이터
camMatrix = calib_data["camMatrix"]
distCoeff = calib_data["distCoeff"]

# 현재 스크립트의 경로를 기반으로 calib_data.npz 로드
calib_file_path = os.path.join(os.path.dirname(__file__), "calib_data.npz")
print(f"📌 Using calibration file: {calib_file_path}")

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        # /camera/color/image_raw 토픽 구독
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

    def image_callback(self, msg):
        # ROS2 이미지 메시지를 OpenCV 포맷으로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aruco 마커 검출
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.1, camMatrix, distCoeff)

            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, camMatrix, distCoeff, rvecs[i], tvecs[i], 0.1)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                self.get_logger().info(f"🔹 ID: {ids[i][0]}, 위치: {tvecs[i]}, 회전: {rvecs[i]}")

        # OpenCV 창에 결과 표시
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
