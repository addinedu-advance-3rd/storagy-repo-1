import os
import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from collections import deque
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import math
from circle.rotate90 import Rotate90
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ArUcoDockingController(Node):
    def __init__(self):
        super().__init__('aruco_dock')

        # ArUco marker settings
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size_1 = 0.1  # marker size in meters
        self.marker_size_2 = 0.2 # marker size in meters

        # 카메라 캘리브레이션 데이터 로드
        # 현재 파이썬 파일 기준의 절대 경로 구하기
        current_dir = os.path.dirname(os.path.abspath(__file__))
        calib_path = os.path.join(current_dir, '..', 'vision', 'calib_data.npz')
        calib_path = os.path.normpath(calib_path)
        try:
            calib_data = np.load(calib_path)
            print (f"✅ Calibration data loaded from {calib_data}")
        except FileNotFoundError as e:
            print (f"❌ File not found: {calib_data}")


        # todo: 알아보기
        self.cmtx = calib_data['camMatrix']    # camera matrix
        self.dist = calib_data['distCoeff']    # distortion coefficients


        # 이미지 처리를 위한 객체 초기화
        self.bridge = CvBridge()
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        # 로봇 이동을 위한 명령 퍼블리셔 생성
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 너무 잦은 요청을 방지하기 위해 잠금 플래그 추가
        self.cmd_vel_locked = False


        # 도킹 활성화 서비스 생성
        self.dock_service = self.create_service(
            Trigger,
            '/dock_robot',  
            self.dock_callback
        )
        self.get_logger().info("도킹 서비스 등록: /dock_robot")

        # 도킹 활성화 플래그
        self.control_active = False

        # 10 Hz 주기로 타이머 콜백 실행
        self.timer = self.create_timer(0.1, self.timer_callback)


        # 휠 오도메트리 구독
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            10
        )
        # 회전 완료 여부 확인을 위한 토픽 구독
        self.rotation_done_sub = self.create_subscription(
            Bool,
            '/rotation_done',
            self.rotation_done_callback,
            10
        )

        # 서비스 클라이언트 추가
        self.rotate_client = self.create_client(Trigger, '/rotate_90_degrees')
        while not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('rotate_90_degrees 서비스를 기다리는 중...')

        print('✅ ArUco docking controller activated.')

        # 회전 상태 추적 플래그
        self.rotation_in_progress = False
        # 첫 번째 목표 도달 플래그
        self.first_goal_reached = False
        # 휠 오도메트리 이동 플래그
        self.odom_control_active = False
        self.odom_moving = False
        self.odom_moving_distance = 0.0
        self.odom_initial_pose = None

        self.docking_complete_pub = self.create_publisher(Bool, '/docking_complete', 10)

        # 디버깅을 위한 로그 추가


    def logger(self, message):
        self.get_logger().info(message)

    # 너무 잦은 요청을 방지하기 위해 잠금 플래그를 이용한 명령 퍼블리셔 추가
    def cmd_vel_publisher(self, linear_x, angular_z):
        if self.cmd_vel_locked:
            self.logger("cmd_vel_locked")
            return
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        timer = self.create_timer(0.3, lambda: self.unlock_cmd_vel(timer))

    def unlock_cmd_vel(self, timer):
        timer.cancel()
        self.cmd_vel_locked = False


    def dock_callback(self, request, response):
        """도킹 서비스 콜백 함수"""
        self.logger("🔄 도킹 서비스 호출 받음!")
        
        # 도킹 제어 활성화
        self.control_active = True
        self.logger("✅ 도킹 제어 활성화됨!")
        
        # 응답 설정
        response.success = True
        response.message = "도킹 제어가 활성화되었습니다."
        
        return response

    def reset_state(self):
        """Reset docking state."""
        self.control_active = False
        self.get_logger().info("🔄 Docking state reset.")

    def image_callback(self, msg):
        """Convert ROS image message to an OpenCV image."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    
    def wheel_odom_callback(self, msg):
        """휠 오도메트리 콜백"""
        if not self.odom_control_active:
            return
        # 휠 오도메트리 데이터 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        dx, dy = 0, 0

        # 초기 위치 none 일때 초기화
        if not self.odom_moving and self.odom_initial_pose is None:
            self.odom_initial_pose = (x,y)
            # 초기 위치 설정 후 이동 시작
            self.odom_moving = True

        else : 
            dx = x - self.odom_initial_pose[0]
            dy = y - self.odom_initial_pose[1]
            self.odom_moving_distance += math.sqrt(dx**2 + dy**2)
            self.logger(f"휠 오도메트리 이동거리: {self.odom_moving_distance:.8f}")
            twist = Twist()

            # 앞으로 가는 거리
            if self.odom_moving_distance < 0.185:
                self.logger("not quite yet, Move Foward")
                twist.linear.x = 0.08
            else : 
                self.logger("STOP MOVING")
                twist.linear.x = 0.0
                self.odom_moving = False
                self.odom_initial_pose = None
                self.logger("휠 오도메트리 이동 완료")
                self.publish_docking_complete()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher(twist.linear.x, twist.angular.z)
            self.logger("--------------------------------")
            self.logger(f"ODOM MOVING DISTANCE : {self.odom_moving_distance:.8f}")
            self.logger(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
            self.logger("--------------------------------")
            self.cmd_vel_publisher(twist.linear.x, twist.angular.z)

    def publish_docking_complete(self):
        """도킹 완료 메시지 게시"""
        msg = Bool()
        msg.data = True
        twist = Twist()

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher(twist.linear.x, twist.angular.z)

        self.docking_complete_pub.publish(msg)
        self.logger("도킹 완료 메시지 게시")
        self.control_active = False
        self.logger("도킹 제어 비활성화")
        

    def timer_callback(self):
        """주기적으로 실행되는 타이머 콜백"""
        if not self.control_active:
            # 제어가 활성화되지 않았으면 아무것도 하지 않음
            return
        
        # 이미지가 없으면 처리하지 않음
        if self.rgb_image is None or self.rgb_image.size == 0:
            self.logger("이미지가 없습니다!")
            return
        
        frame = self.rgb_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        if ids is not None and self.control_active:
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            best_tvec = None
            best_rvec = None
            best_id = None
            min_distance = float('inf')
            
            # Select the closest marker
            for i in range(len(ids)):
                # 마커 ID에 따라 적절한 마커 크기 선택
                marker_size = self.marker_size_2 if ids[i] == 4 else self.marker_size_1
                
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, self.cmtx, self.dist)
                frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)
                print (f"rvec: {rvec}")
                print (f"tvec: {tvec}")
                corner = corners[i].reshape((4,2))
                x_coord = int(corner[0][0])
                y_coord = int(corner[0][1])

                # rvec → 회전 행렬 변환
                R_matrix, _ = cv2.Rodrigues(rvec)
                # 단순히 yaw (Z축 회전)만 고려 (회전 행렬에서 yaw는 arctan2(R[1,0], R[0,0])로 계산)
                yaw = math.degrees(math.atan2(R_matrix[1, 0], R_matrix[0, 0]))
                roll = math.degrees(math.atan2(R_matrix[2, 1], R_matrix[2, 2]))
                pitch = math.degrees(math.atan2(R_matrix[2, 0], R_matrix[2, 2]))

                self.logger(f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")
                # tvec: (1, 1, 3) 배열이므로 스칼라 값 추출
                tvec_scalar = tvec[0][0]
                # 일반적으로 전진 거리는 카메라 좌표계에서 Z축 (앞 방향)
                distance = tvec_scalar[2]

                self.logger(f"yaw: {yaw:.2f}, distance: {distance:.2f}")

                # 회전 중이면 마커 처리 건너뛰기
                if self.rotation_in_progress:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    continue
                
                if ids[i] == 0 and self.first_goal_reached == False:
                    if distance < 0.85: 
                        # 1차 마커와 충분히 가까워지면 회전 명령 직접 실행
                        self.perform_90_degree_rotation()
                        self.first_goal_reached = True
                    else : 
                        self.logger(f"✅ Marker ID: {ids[i]}")
                        self.logger("1차 방향 정렬 시작")
                        if 180 - abs(yaw) > 20 : 
                            if yaw > 0 : 
                                twist.angular.z = 0.2
                            else : 
                                twist.angular.z = -0.2
                        else :
                            if 180 - abs(yaw) > 4 :
                                if yaw > 0 : 
                                    twist.angular.z = 0.08
                                else : 
                                    twist.angular.z = -0.08
                            else : 
                                twist.angular.z = 0.0
                                self.logger("1차 직선 이동 시작")
                                twist.linear.x = 0.08
                elif ids[i] == 4 and self.first_goal_reached and not self.rotation_in_progress:
                    if distance < 0.3:
                        self.logger('2차 목표 도착')
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        # 마지막으로 ODOM 으로 전진
                        self.odom_control_active = True
                    else :
                        self.logger('2차 방향정렬 시작')
                        if 180 - abs(yaw) > 20 : 
                            self.logger('절대값 비교시 20이상 차이남')
                            if yaw > 0 : 
                                twist.angular.z = 0.23
                            else : 
                                twist.angular.z = -0.2
                        else :
                            # 수동 PID 제어를 위한 조건문
                            self.logger('절대값 비교시 20이하로 차이남')
                            if 180 - abs(yaw) > 4 :
                                if yaw > 0 : 
                                    twist.angular.z = 0.12
                                else : 
                                    twist.angular.z = -0.09
                            else : 
                                self.logger('절대값 비교시 4이하로 차이남')
                                twist.angular.z = 0.0
                                self.logger('2차 직선 이동 시작')
                                twist.linear.x = 0.15
                            
                else :
                    if ids[i] == 4 and self.first_goal_reached == False:
                        self.logger("NOT RELEVANT MARKER")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.14
                    elif ids[i] == 0 and self.first_goal_reached == True:
                        self.logger("NOT RELEVANT MARKER")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.13
        
        
        elif not self.rotation_in_progress and self.control_active:
            twist.linear.x = 0.0
            twist.angular.z = 0.18
            self.logger("Marker not found")

        # 디버깅을 위한 이미지 표시
        cv2.imshow("ArUco Docking", frame)
        cv2.waitKey(1)

        

        self.logger(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
        
        # 명령 퍼블리시
        self.logger("--------------------------------")
        self.logger("PUBLISHING TWIST")
        self.logger(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
        self.logger("--------------------------------")
        self.cmd_vel_publisher(twist.linear.x, twist.angular.z) if not self.odom_control_active else self.logger("ODOM CONTROL ACTIVE, DONT MOVE")
        


    def __del__(self):
        cv2.destroyAllWindows()

    def perform_90_degree_rotation(self):
        """서비스를 통해 90도 회전 요청"""
        # 회전 시작 플래그 설정
        self.rotation_in_progress = True
        
        request = Trigger.Request()
        future = self.rotate_client.call_async(request)
        self.logger("90도 회전 요청 전송")
        
        # 비동기 호출이므로 결과를 기다리지 않음
        # 회전이 완료되면 rotation_done 토픽을 통해 알림 받음

    def rotation_done_callback(self, msg):
        """회전 완료 토픽 콜백"""
        self.logger(f"회전 상태 메시지 수신: {msg.data}")
        
        if msg.data:  # True일 때 (회전 완료)
            self.logger("회전 완료 메시지 수신 - 회전 완료!")
            self.rotation_in_progress = False
        else:  # False일 때 (회전 시작)
            self.logger("회전 시작 메시지 수신 - 회전 시작!")
            self.rotation_in_progress = True

def main(args=None):
    rp.init(args=args)
    node = ArUcoDockingController()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()