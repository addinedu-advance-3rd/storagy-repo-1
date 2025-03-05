import os
import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

class ArUcoDockingController(Node):
    def __init__(self):
        super().__init__('aruco_docking_controller')

        # ArUco 마커 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # 50mm = 0.05m

        # 캘리브레이션 데이터 로드
        current_dir = os.path.dirname(os.path.abspath(__file__))
        calib_data_path = os.path.join(current_dir, "../vision/calib_data.npz")

        # 현재 실행 중인 스크립트의 디렉토리 가져오기
        # print(f"current_dir: {current_dir}")

        # 절대경로로 `calib_data.npz` 경로 설정
        # print(f"calib_data_path: {calib_data_path}")
        


        try : 
            calib_data = np.load(calib_data_path)
        except FileNotFoundError as e:
            try : 
                calib_data = np.load("circle/src/circle/vision/calib_data.npz")
            except FileNotFoundError as e:
                raise FileNotFoundError(f"❌ 파일이 존재하지 않습니다: {calib_data_path}")
                
        self.cmtx = calib_data['camMatrix']  # 카메라 행렬
        self.dist = calib_data['distCoeff']  # 왜곡 계수

        # CvBridge 초기화
        self.bridge = CvBridge()

        # ROS 이미지 구독 설정
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 카메라 토픽
            self.image_callback,
            10
        )
        # ROS 제어 명령 퍼블리셔 (로봇 이동 명령)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ArUco 마커 도킹 활성화 서비스
        self.service = self.create_service(Trigger, 'dock_robot', self.service_callback)

        # 도킹 활성화 상태
        self.control_active = False

        # 타이머 설정 (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)  # 기본 이미지

        # 최소 속도 및 PID 게인 설정
        self.MIN_LINEAR_SPEED = 0.03  # 최소 선속도 (m/s)
        self.MIN_ANGULAR_SPEED = 0.03  # 최소 각속도 (rad/s)

        # 도킹 목표값 및 PID 게인
        self.target_tvec_x = 0.00  # 좌우 정렬 목표 (0이면 마커 중앙)
        self.target_tvec_z = 0.01  # 목표 거리 (30cm)
        self.target_rvec_y = 0.00  # 회전 목표 (0° 유지)

        self.Kp_x = 0.1   # 좌우 정렬 제어 게인
        self.Kp_z = 0.4   # 전진 속도 제어 게인
        self.Kp_y = 0.05  # 회전 속도 제어 게인
        self.prev_angular_z = 0.0
        self.depth_in_use = False
        self.FINAL_DEPTH_STOP = 0.01  # 최종 도킹 거리 (1cm)


        self.prev_angle_error = 0.0

        self.get_logger().info('✅ ArUco 도킹 컨트롤러 활성화됨.')
        print("✅ ArUco 도킹 컨트롤러 활성화됨.")
        print("✅ ArUco 도킹 컨트롤러 활성화 flush.", flush=True)


    def service_callback(self, request, response):
        """도킹 활성화 서비스 콜백"""
        self.control_active = not self.control_active
        if self.control_active:
            self.get_logger().info("🚀 ArUco 도킹 활성화됨.")
            response.success = True
            response.message = "ArUco 도킹이 활성화되었습니다."
        else:
            self.get_logger().info("🛑 ArUco 도킹 비활성화됨.")
            response.success = True
            response.message = "ArUco 도킹이 비활성화되었습니다."
        return response

    def reset_state(self):
        """도킹 상태 초기화"""
        self.prev_angular_z = 0.0
        self.control_active = False
        self.get_logger().info("🔄 도킹 상태 초기화 완료.")

    def normalized_angle_error(self, current_angle, target_angle):
        """현재 각도와 목표 각도의 오차를 계산하고 [-180, 180] 범위로 정규화"""
        error = (current_angle - target_angle + 180) % 360 - 180
        return error


    def image_callback(self, msg):
        """ROS 이미지 메시지를 OpenCV 이미지로 변환"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        # print("🔄 image_callback 호출됨")
        frame = self.rgb_image.copy()
        # print("🔄 이미지 복사 완료")
        best_id = None
        angle_error = 0.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        twist = Twist()

        # print("🔄 마커 찾으러 가는중 ")
        if ids is not None:
            best_tvec = None  # 가장 적절한 마커 위치 정보
            best_rvec = None

            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.cmtx, self.dist)

                # ID 20 (작은 마커)가 있으면 우선 사용
                # if ids[i] == 20:
                if ids[i] is not None:
                    best_tvec = tvec
                    best_rvec = rvec
                    best_id = ids[i]
                    break  # 작은 마커가 감지되면 즉시 사용

                # ID 10 (큰 마커)만 감지되었을 때
                if best_tvec is None and ids[i] == 10:
                    best_tvec = tvec
                    best_rvec = rvec
                    best_id = 10

            if best_tvec is not None:
                angle_error = self.normalized_angle_error(np.rad2deg(np.squeeze(best_rvec))[1], 0.0)
                print(f"angle_error: {angle_error}")
                self.process_docking(best_tvec, best_rvec, best_id, twist, angle_error)



        self.cmd_vel_pub.publish(twist)
        cv2.imshow("ArUco Docking", frame)
        self.prev_angular_z = twist.angular.z
        self.prev_angle_error = angle_error
        print(f"twist.angular.z: {twist.angular.z}")
        self.get_logger().info(f"marker_id: {best_id}")


    #     self.get_logger().info("🛑 도킹 중지")
    #     self.reset_state()

        cv2.waitKey(1)

    
    def process_docking(self, tvec, rvec, marker_id, twist, angle_error):
        tvec = np.squeeze(tvec)
        rvec = np.rad2deg(np.squeeze(rvec))

        # 도킹 목표값 설정
        self.target_tvec_x = 0.00
        self.target_tvec_z = 0.30
        self.target_rvec_y = 0.00


        diff_x = tvec[0] - self.target_tvec_x
        diff_z = tvec[2] - self.target_tvec_z
        diff_yaw = rvec[1] - self.target_rvec_y

        tolerance_x = 0.01
        tolerance_z = 0.01
        tolerance_yaw = 1.0

        if abs(diff_x) < tolerance_x and abs(diff_z) < tolerance_z and abs(diff_yaw) < tolerance_yaw:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"✅ 도킹 완료 (마커 ID: {marker_id})")
        else:
            twist.linear.x = max(min(self.Kp_z * diff_z, 0.3), -0.3)
            twist.angular.z = max(min(self.Kp_x * diff_x + self.Kp_y * diff_yaw, 0.08), -0.08)


        if self.prev_angle_error >= angle_error : 
            twist.angular.z = twist.angular.z * 0.8

        # self.get_logger().info(
        #     f"🔹 마커 ID: {marker_id}, tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}"
        # )
        

        # self.cmd_vel_pub.publish(twist)
        # cv2.imshow("ArUco Docking", frame)
        # if cv2.waitKey(1) == 27:
        #     self.get_logger().info("🛑 도킹 중지")
        #     self.reset_state()

    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rp.init(args=args)
    node = ArUcoDockingController()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()