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
        calib_data = np.load("circle/src/circle/vision/calib_data.npz")
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
        # Depth 이미지 구독
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # Depth 카메라 토픽
            self.depth_callback,
            10
        )
        self.depth_image = None


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

    # Depth 이미지 구독
    def depth_callback(self, msg):
        """Depth 이미지 콜백 함수"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


    def reset_state(self):
        """도킹 상태 초기화"""
        self.prev_angular_z = 0.0
        self.control_active = False
        self.get_logger().info("🔄 도킹 상태 초기화 완료.")

    def image_callback(self, msg):
        """ROS 이미지 메시지를 OpenCV 이미지로 변환"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def get_distance_from_depth(self):
        """Depth 카메라에서 벽까지의 평균 거리 계산"""
        if self.depth_image is None:
            return None  # Depth 데이터가 아직 없음

        # 중앙 영역(ROI) 설정 (이미지 중앙 100x100 픽셀)
        h, w = self.depth_image.shape
        center_x, center_y = w // 2, h // 2
        roi_size = 75  # ROI 영역 크기
        roi = self.depth_image[center_y - roi_size // 2 : center_y + roi_size // 2,
                            center_x - roi_size // 2 : center_x + roi_size // 2]

        # NaN 값 제거 후 평균 거리 계산
        valid_values = roi[roi > 0]
        if valid_values.size == 0:
            self.get_logger().warn("⚠️ Depth 데이터 없음: 정지 유지")
            return None  # 유효한 거리 값 없음

        mean_distance = np.mean(valid_values) / 1000.0  # mm → meter 변환
        self.get_logger().info(f"📏 Depth 거리: {mean_distance:.4f}m")
        return mean_distance


    def normalized_angle_error(self, current_angle, target_angle):
        """현재 각도와 목표 각도의 오차를 계산하고 [-180, 180] 범위로 정규화"""
        error = (current_angle - target_angle + 180) % 360 - 180
        return error

    def timer_callback(self):
        """주기적으로 실행되는 도킹 제어 로직"""
        corners, ids, _ = cv2.aruco.detectMarkers(self.rgb_image, self.aruco_dict, parameters=self.parameters)
        twist = Twist()
        frame = self.rgb_image.copy()

        angle_error = 0.0
        if self.depth_in_use:
            depth_distance = self.get_distance_from_depth()

            if depth_distance is not None:
                self.get_logger().info(f"📏 Depth 거리: {depth_distance:.4f}m")
                if depth_distance <= self.FINAL_DEPTH_STOP:
                    # 최종 Depth 기반 도킹 완료 (STEP 3)
                    self.get_logger().info("🎉 Depth 기반 최종 도킹 완료")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    self.reset_state()
                    return
                else:
                    # Depth 거리 기반 전진
                    self.depth_in_use = True
                    twist.linear.x = max(min(self.Kp_z * (depth_distance - self.FINAL_DEPTH_STOP), 0.05), self.MIN_LINEAR_SPEED)
                    twist.angular.z = 0.0
                    self.get_logger().info("➡️ Depth 카메라 기반 전진 중")
            else:
                self.get_logger().warn("⚠️ Depth 데이터 없음: 정지 유지")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            #  아루코마커를 찾았다
            if ids is not None:
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[0], self.marker_size, self.cmtx, self.dist)
                frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)

                rvec_deg = np.rad2deg(np.squeeze(rvec))
                tvec = np.squeeze(tvec)

                diff_x = tvec[0] - self.target_tvec_x
                diff_z = tvec[2] - self.target_tvec_z
                angle_error = self.normalized_angle_error(rvec_deg[1], self.target_rvec_y)


                # 허용 범위 설정
                # CLOSE_DISTANCE = 0.10       # ArUco 기반 거리 (10cm)
                CLOSE_DISTANCE = 0.01       # ArUco 기반 거리 (10cm)
                ALIGNMENT_TOLERANCE = 2.0   # 각도 오차 (2도)
                
                if abs(diff_z) <= CLOSE_DISTANCE:
                    # 충분히 가까워졌을 때 (STEP 1)
                    self.get_logger().info("✅ 충분히 가까움: 각도 정렬만 수행")
                    # self.depth_in_use = False

                    if 180.0 - abs(angle_error) <= ALIGNMENT_TOLERANCE:
                        # 각도 정렬 완료 후 Depth 전진 모드 (STEP 2)
                        self.depth_in_use = True
                    else:
                        # 각도 오차가 아직 크다면 각도만 제어
                        twist.linear.x = 0.0
                        if not self.depth_in_use:
                            twist.angular.z = self.Kp_y * angle_error   
                            twist.angular.z = max(min(twist.angular.z, 0.08), -0.08)
                        self.prev_angular_z = twist.angular.z
                        self.get_logger().info(f"🔄 각도 정렬 중: 오차={angle_error:.2f}°")
                else:
                    # 멀리 있을 때 일반 ArUco 도킹 제어 (STEP 4)
                    print("🔄 멀리 있을 때 일반 ArUco 도킹 제어")
                    self.depth_in_use = False
                    linear_x = self.Kp_z * diff_z
                    linear_x = np.clip(linear_x, -0.2, 0.2)


                    # angular_z = self.Kp_x * diff_x + self.Kp_y * angle_error + sway_adjust
                    angular_z = self.Kp_x * diff_x + self.Kp_y * angle_error
                    angular_z = np.clip(angular_z, -0.2, 0.2)

                    twist.linear.x = linear_x if abs(linear_x) >= self.MIN_LINEAR_SPEED else np.sign(linear_x) * self.MIN_LINEAR_SPEED
                    twist.angular.z = angular_z if abs(angular_z) >= self.MIN_ANGULAR_SPEED else np.sign(angular_z) * self.MIN_ANGULAR_SPEED

                    self.get_logger().info(f"🚗 ArUco 접근 중: 거리={diff_z:.2f}m, 각도={angle_error:.2f}°")
                    self.get_logger().info(f"twist.linear.x: {twist.linear.x}, twist.angular.z: {twist.angular.z}")
            # 아루코 마커를 찾지 못했다. 최소 회전 속도 적용
            else:
                if not self.depth_in_use:
                    print("🔄 멀리 있을 때 일반 ArUco 도킹 제어")
                    if self.prev_angular_z != 0.0:
                        if self.prev_angular_z < 0.2:
                            twist.angular.z = 0.2
                        else:
                            twist.angular.z = self.prev_angular_z * 0.8
                    else:
                        twist.angular.z = 0.5

        self.prev_angular_z = twist.angular.z
        print(f"twist.angular.z: {twist.angular.z}")
        self.cmd_vel_pub.publish(twist)
        cv2.imshow("ArUco Docking", frame)
        if cv2.waitKey(1) == 27:
            self.get_logger().info("🛑 도킹 중지")
            self.reset_state()

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




# #     def timer_callback(self):
#         """주기적으로 실행되는 도킹 제어 로직"""
#         # ArUco 마커 검출
#         corners, ids, _ = cv2.aruco.detectMarkers(self.rgb_image, self.aruco_dict, parameters=self.parameters)
#         twist = Twist()  # Twist 메시지 초기화
#         frame = self.rgb_image.copy()
#         linear_x = 0.0
#         angular_z = 0.0


#         if ids is not None:
#             if not self.control_active:
#                 self.get_logger().info("🚀 ArUco 마커 감지! 도킹 시작")
#                 self.control_active = True  # 도킹 자동 활성화

#             frame = cv2.aruco.drawDetectedMarkers(self.rgb_image.copy(), corners, ids)
#             for i, marker_id in enumerate(ids):
#                 rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
#                     corners[i], self.marker_size, self.cmtx, self.dist)
#                 frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)
#                 # rvec, tvec 압축 및 단위 변환
#                 rvec = np.rad2deg(np.squeeze(rvec))  # 라디안 → 도
#                 tvec = np.squeeze(tvec)             # 3D 위치 정보

#                 # 각 오차 계산 (회전각은 정규화)
#                 diff_x = tvec[0] - self.target_tvec_x
#                 diff_z = tvec[2] - self.target_tvec_z
#                 print (f"diff_x: {diff_x}, diff_z: {diff_z}")
#                 angle_error = self.normalize_angle(rvec[1] - self.target_rvec_y)

#                 # 오차 허용 범위 (tolerance)
#                 tolerance_x = 0.1  # 10cm
#                 tolerance_z = 0.1  # 10cm
#                 tolerance_yaw = 2.0  # 2도

#                 # CLOSE_ENOUGH_DISTANCE = 0.08  # Depth Camera 모드 전환 기준 거리 (8cm)
#                 # ALIGNMENT_TOLERANCE = 1.0  # 각도 오차 2도 이내면 정렬 완료

#                 # if abs(diff_z) >= tolerance_z:
#                 #     linear_x = self.Kp_z * diff_z
#                 #     if abs(linear_x) < self.MIN_LINEAR_SPEED:
#                 #         linear_x = self.MIN_LINEAR_SPEED if linear_x > 0 else -self.MIN_LINEAR_SPEED
#                 #     # print(f"[LOG] diff_z too small ({diff_z:.4f})")
#                 #     twist.linear.x = linear_x

#                 # 가까우면 회전 제어 
#                 if abs(diff_z) <= tolerance_z:

#                     self.get_logger().info("✅ 충분히 가까워짐, Depth Camera 모드로 전환")
#                     print(f"angle_error: {angle_error}")
                    
#                     # 각도 정렬이 완료되었으면 도킹 종료
#                     if abs(angle_error) < ALIGNMENT_TOLERANCE:
#                         self.get_logger().info("✅ 최종 정렬 완료, 도킹 종료")
#                         twist.linear.x = 0.0
#                         twist.angular.z = 0.0
#                         self.cmd_vel_pub.publish(twist)
#                         self.reset_state()
#                         return
                    
#                     # 아직 각도가 틀어져 있으면 회전만 수행
#                     twist.linear.x = 0.0
#                     twist.angular.z = self.Kp_y * angle_error
#                     twist.angular.z = max(min(twist.angular.z, 0.08), -0.08)
#                     print("충분히 가까움 회전 제어만 수행")
#                     print(f"twist.angular.z: {twist.angular.z}")
#                     self.cmd_vel_pub.publish(twist)
#                     # return

#                     # DEPTH 운전
#                     depth_distance = self.get_distance_from_depth()     
#                     if depth_distance is not None:
#                         if depth_distance < 0.02:  # Depth 기준 8cm 이내면 도킹 완료
#                             twist.linear.x = 0.0
#                             twist.angular.z = 0.0
#                             self.cmd_vel_pub.publish(twist)
#                             self.get_logger().info("✅ Depth 기반 도킹 완료!")
#                             self.reset_state()
#                             return

#                         # Depth 거리 기반 속도 조절
#                         linear_x = self.Kp_z * (depth_distance - 0.08)
#                         linear_x = max(min(linear_x, 0.3), -0.3)
#                         twist.linear.x = linear_x

#                 # if (abs(diff_x) < tolerance_x and 
#                 #     abs(diff_z) < tolerance_z and 
#                 #     abs(angle_error) < tolerance_yaw):
#                 #     twist.linear.x = 0.0
#                 #     twist.angular.z = 0.0
#                 #     self.cmd_vel_pub.publish(twist)
#                 #     self.get_logger().info("✅ 도킹 완료, 로봇 정지")
#                 #     self.reset_state()
#                 #     break  # 하나라도 완료되면 더 이상 제어하지 않음
#                 # else:
#                     # 전진 제어: diff_z가 충분히 크면 속도 적용
#                     # 작으면 DEPTH 운전으로 전환


#                         # linear_x = 0.001  # 강제 전진
#                         # linear_x = 0.02  # 강제 전진
                    
#                 else: 
#                     # 회전 제어: diff_x와 angle_error를 함께 고려
#                     if abs(diff_x) >= tolerance_x or abs(angle_error) >= tolerance_yaw:
#                         print (f"diff_x: {diff_x}, angle_error: {angle_error}")
#                         print (f"self.Kp_x: {self.Kp_x}, self.Kp_y: {self.Kp_y}")
#                         angular_z = self.Kp_x * diff_x + self.Kp_y * angle_error
#                         if abs(angular_z) < self.MIN_ANGULAR_SPEED:
#                             angular_z = self.MIN_ANGULAR_SPEED if angular_z > 0 else -self.MIN_ANGULAR_SPEED
#                     else:
#                         angular_z = 0.0

#                     twist.linear.x = max(min(linear_x, 0.3), -0.3)
#                     twist.angular.z = max(min(angular_z, 0.3), -0.3)
#                     self.prev_angular_z = twist.angular.z

#                     self.get_logger().info(
#                         f"tvec_x={tvec[0]:.2f}, tvec_z={tvec[2]:.2f}, rvec_y={rvec[1]:.2f}, "
#                         f"linear_x={linear_x:.2f}, angular_z={angular_z:.2f}"
#                     )
#         else:
#             self.get_logger().warn("⚠ ArUco 마커 감지 실패.")
#             twist.angular.z = -self.prev_angular_z * 0.8 if self.prev_angular_z != 0.0 else 0.1

#         self.cmd_vel_pub.publish(twist)
#         cv2.imshow("ArUco Docking", frame)
#         if cv2.waitKey(1) == 27:
#             self.get_logger().info("🛑 도킹 중지")
#             self.reset_state()