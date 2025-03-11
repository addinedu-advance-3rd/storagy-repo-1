import os
import sys
import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_srvs.srv import Trigger  # 표준 서비스 메시지

class ArUcoMarkerController(Node):
    def __init__(self):
        super().__init__('aruco_marker_controller')

        # ArUco 마커 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # 50mm = 0.05m

        # 캘리브레이션 데이터 로드

        # calib_data = np.load("circle/src/circle/vision/calib_data.npz")
        calib_data = np.load("/home/storagy/circle/src/circle/vision/calib_data.npz")
        self.cmtx = calib_data['camMatrix']  # 카메라 행렬
        self.dist = calib_data['distCoeff']  # 왜곡 계수
        # CvBridge 초기화
        self.bridge = CvBridge()

        # ROS 이미지 구독 설정
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # ROS 카메라 토픽 이름
            self.image_callback,
            1
        )

        # ROS 제어 명령 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 서비스 서버 생성
        self.service = self.create_service(Trigger, 'home', self.service_callback)

        # ArUco 마커 제어 활성화 상태
        self.control_active = False

        # 타이머 설정
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)  # 기본 이미지

        # 제어 목표값 및 게인 설정
        self.target_tvec_x = 0.00
        self.target_tvec_z = 0.20
        self.target_rvec_y = 0.06
        self.Kp_x = 0.1  # tvec_x 제어 게인
        self.Kp_z = 0.4  # tvec_z 제어 게인
        self.Kp_y = 0.05  # rvec_y 제어 게인
        self.prev_angular_z = 0.0

        self.get_logger().info('Aruco marker control service server activated.')

    def service_callback(self, request, response):
        """서비스 콜백 함수"""
        self.control_active = not self.control_active
        if self.control_active:
            self.get_logger().info("ArUco Marker Control 활성화됨.")
            response.success = True
            response.message = "ArUco Marker Control이 활성화되었습니다."
        else:
            self.get_logger().info("ArUco Marker Control 비활성화됨.")
            response.success = True
            response.message = "ArUco Marker Control이 비활성화되었습니다."
        
        return response
    
    def reset_state(self):
        """상태 초기화 메서드"""
        self.prev_angular_z = 0.0  # 이전 회전 값 초기화
        self.control_active = False  # 제어 비활성화
        self.get_logger().info("상태가 초기화되었습니다.")

    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        print(f"control_active: {self.control_active}")
        if not self.control_active:
            return  # 제어 비활성화 시 실행하지 않음

        # ArUco 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(self.rgb_image, self.aruco_dict, parameters=self.parameters)

        twist = Twist()  # Twist 메시지 초기화

        if ids is not None:  # 마커가 감지된 경우
            frame = cv2.aruco.drawDetectedMarkers(self.rgb_image.copy(), corners, ids)
            for i, marker_id in enumerate(ids):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.cmtx, self.dist)
                frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)
                rvec = np.rad2deg(np.squeeze(rvec))
                tvec = np.squeeze(tvec)

                text1 = f"tvec_x: {tvec[0]:.2f} m"
                text2 = f"tvec_z: {tvec[2]:.2f} m"
                text3 = f"rvec_y: {rvec[1]:.2f} deg"

                cv2.putText(frame, text1, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)  # 첫 번째 텍스트
                cv2.putText(frame, text2, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)  # 두 번째 텍스트
                cv2.putText(frame, text3, (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)  # 세 번째 텍스트

                diff_x = tvec[0] - self.target_tvec_x
                diff_z = tvec[2] - self.target_tvec_z
                diff_yaw = rvec[1] - self.target_rvec_y

                tolerance_x = 0.05
                tolerance_z = 0.01
                tolerance_yaw = 3.8

                if abs(diff_x) < tolerance_x and abs(diff_z) < tolerance_z and abs(diff_yaw) < tolerance_yaw:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("목표 위치에 도달, 호출대기 상태로 돌아갑니다")
                    self.reset_state()
                    
                else:
                    linear_x = self.Kp_z * diff_z
                    angular_z = self.Kp_x * diff_x + self.Kp_y * diff_yaw
                    twist.linear.x = max(min(linear_x, 0.3), -0.3)
                    twist.angular.z = max(min(angular_z, 0.2), -0.2)
                    self.prev_angular_z = twist.angular.z

                    self.get_logger().info(
                        f"tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}, "
                        f"linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
                    )
        else:
            self.get_logger().warn("ArUco 마커가 감지되지 않음.")
            twist.angular.z = -self.prev_angular_z * 0.8 if self.prev_angular_z != 0.0 else 0.1

        self.cmd_vel_pub.publish(twist)
        cv2.imshow("ArUco Marker Control", frame if ids is not None else self.rgb_image)
        if cv2.waitKey(1) == 27:  # ESC 키
            self.get_logger().info("호출대기 상태로 돌아갑니다")
            self.reset_state()

    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rp.init(args=args)
    node = ArUcoMarkerController()
    node.get_logger().info("ArUco Marker Controller Node started.")
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램이 종료되었습니다.")
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
    main()
