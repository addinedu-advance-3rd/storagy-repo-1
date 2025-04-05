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
import tf_transformations
from std_msgs.msg import Bool
# Simple 1D Kalman Filter for smoothing the angle measurement
class KalmanFilter:
    def __init__(self, initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0):
        self.x = initial_value         # state estimate
        self.P = error_estimate        # error covariance estimate
        self.Q = process_noise         # process noise covariance
        self.R = measurement_noise     # measurement noise covariance

    def update(self, measurement):
        # Calculate Kalman Gain
        K = self.P / (self.P + self.R)
        # Update the state estimate with the measurement
        self.x = self.x + K * (measurement - self.x)
        # Update the error covariance
        self.P = (1 - K) * self.P + self.Q
        return self.x

class ArUcoDockingController(Node):
    def __init__(self):
        super().__init__('aruco_dock')

        # ArUco marker settings
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size_1 = 0.1  # marker size in meters
        self.marker_size_2 = 0.2 # marker size in meters
        self.effective_Kp_y = 0.0
        
        # Load calibration data
        try:
            # package_dir = get_package_share_directory('circle')
            calib_data = np.load( '/home/storagy/storagy-repo-1/circle/src/circle/vision/calib_data.npz')
            print (f"✅ Calibration data loaded from {calib_data}")
        except FileNotFoundError as e:
            print (f"❌ File not found: {calib_data}")

        self.cmtx = calib_data['camMatrix']  # camera matrix
        self.dist = calib_data['distCoeff']    # distortion coefficients

        # Initialize CvBridge
        self.bridge = CvBridge()

        # ROS image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        # ROS command publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ArUco docking activation service
        self.dock_service = self.create_service(
            Trigger,
            '/dock_robot',  # 서비스 이름이 '/dock_robot'인지 확인
            self.dock_callback
        )
        self.get_logger().info("도킹 서비스 등록: /dock_robot")

        # Docking active flag
        self.control_active = False

        # Timer setup (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Minimum speeds and PID gains
        self.MIN_LINEAR_SPEED = 0.03  # m/s
        self.MIN_ANGULAR_SPEED = 0.03  # rad/s

        # Docking target values and PID gains
        self.target_tvec_x = 0.00  # target lateral alignment (0 for marker center)
        self.target_tvec_z = 0.01  # target distance (in meters)
        self.target_rvec_y = 0.00  # target rotation (maintain 0°)

        self.Kp_x = 0.1   # lateral control gain
        self.Kp_z = 0.4   # forward speed control gain
        self.Kp_y = 0.05  # angular control gain (base gain)
        self.kd_y = 0.01  # derivative gain
        self.prev_distance = 0.0

        # State variables for control
        self.current_angle = 0.0
        self.prev_angular_z = 0.0
        self.prev_linear_x = 0.0 
        self.prev_angle_error = 0.0
        self.prev_mean_error = 0.0
        self.filtered_angle = 0.0
        self.prev_filtered_angle = 0.0
        self.first_goal_reached = False
        # For filtering and trend estimation
        self.error_history = deque(maxlen=5)
        self.kalman_filter = KalmanFilter(initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0)
        self.rotation_done_sub = self.create_subscription(
            Bool,
            '/rotation_done',
            self.rotation_done_callback,
            10
        )
        # Parameters for adaptive controller gain adjustment
        self.adaptive_alpha = 0.05  # scaling factor for the trend
        self.min_gain = 0.01       # minimum angular gain
        self.max_gain = 0.1        # maximum angular gain

        # Tolerances
        self.tolerance_x = 0.15
        self.tolerance_z = 0.01
        # self.tolerance_yaw = 10.0

        # 서비스 클라이언트 추가
        self.rotate_client = self.create_client(Trigger, '/rotate_90_degrees')
        while not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('rotate_90_degrees 서비스를 기다리는 중...')

        print('✅ ArUco docking controller activated.')

        # 회전 상태 추적 플래그
        self.rotation_in_progress = False
        self.first_goal_reached = False
        self.odom_moving = False
        self.odom_moving_distance = 0.0
        self.odom_initial_pose = None

        # 디버깅을 위한 로그 추가
        self.get_logger().info("회전 완료 토픽 구독 시작: /rotation_done")

    def dock_callback(self, request, response):
        """도킹 서비스 콜백 함수"""
        self.get_logger().info("🔄 도킹 서비스 호출 받음!")
        
        # 도킹 제어 활성화
        self.control_active = True
        self.get_logger().info("✅ 도킹 제어 활성화됨!")
        
        # 응답 설정
        response.success = True
        response.message = "도킹 제어가 활성화되었습니다."
        
        return response

    def reset_state(self):
        """Reset docking state."""
        self.prev_angular_z = 0.0
        self.control_active = False
        self.get_logger().info("🔄 Docking state reset.")

    def normalized_angle_error(self, current_angle, target_angle):
        """Compute normalized angle error in the range [-180, 180]."""
        error = (current_angle - target_angle + 180) % 360 - 180
        return error

    def rotation_matrix_to_euler_angles(self, R_matrix):
        """
        Convert a rotation matrix to Euler angles.
        Returns [roll, pitch, yaw] in degrees.
        """
        r = R.from_matrix(R_matrix)
        return r.as_euler('xyz', degrees=True)

    def image_callback(self, msg):
        """Convert ROS image message to an OpenCV image."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def move_forward_using_wheel_odom(self):
        """휠 오도메트리를 사용하여 전진 이동"""
        # 휠 오도메트리 데이터 구독

        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            10
        )
    
    def wheel_odom_callback(self, msg):
        """휠 오도메트리 콜백"""
        self.get_logger().info("휠 오도메트리 데이터 수신")
        # 휠 오도메트리 데이터 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        dx, dy = 0, 0
        # 휠 오도메트리 데이터 출력
        self.get_logger().info(f"휠 오도메트리 데이터: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        # move forward using wheel odom

        if not self.odom_moving and self.odom_initial_pose is None:
            self.odom_initial_pose = (x,y)
            self.odom_moving_distance = 0.0
            self.odom_moving = True

        else : 
            dx = x - self.odom_initial_pose[0]
            dy = y - self.odom_initial_pose[1]
            self.odom_moving_distance = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f"휠 오도메트리 이동거리: {self.odom_moving_distance:.2f}")
            twist = Twist()

            if self.odom_moving_distance < 0.013:
                print("not quite yet, Move Foward")
                twist.linear.x = 0.08
            else : 
                print("STOP MOVING")
                twist.linear.x = 0.0
                self.odom_moving = False
                self.odom_moving_distance = 0.0
                self.odom_initial_pose = None
                self.get_logger().info("휠 오도메트리 이동 완료")
            print("--------------------------------")
            print(f"ODOM MOVING DISTANCE : {self.odom_moving_distance:.2f}")
            print(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
            print("--------------------------------")
            self.cmd_vel_pub.publish(twist)

    def timer_callback(self):
        """주기적으로 실행되는 타이머 콜백"""
        if not self.control_active:
            # 제어가 활성화되지 않았으면 아무것도 하지 않음
            return
        
        self.get_logger().info("도킹 제어 활성화 상태: 마커 추적 중...")
        
        # 이미지가 없으면 처리하지 않음
        if self.rgb_image is None or self.rgb_image.size == 0:
            self.get_logger().warn("이미지가 없습니다!")
            return
        
        frame = self.rgb_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        #init TWIST


        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        print (f"ids: {ids}")

        if ids is not None and self.control_active:
            print("########################################################")
            print("FOUND MARKER")
            print("FOUND MARKER")
            print("FOUND MARKER")
            print("FOUND MARKER")
            print("FOUND MARKER")
            print("########################################################")
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

                print (f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")
                # tvec: (1, 1, 3) 배열이므로 스칼라 값 추출
                tvec_scalar = tvec[0][0]
                # 일반적으로 전진 거리는 카메라 좌표계에서 Z축 (앞 방향)
                distance = tvec_scalar[2]

                print (f"yaw: {yaw:.2f}, distance: {distance:.2f}")

                # 회전 중이면 마커 처리 건너뛰기
                if self.rotation_in_progress:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    continue
                
                if ids[i] == 0 and self.first_goal_reached == False:
                    if distance < 0.85: 
                        print ("회전 준비")
                        # 회전 명령 직접 실행
                        self.perform_90_degree_rotation()
                        self.first_goal_reached = True
                        
                    else : 
                        print (f"✅ Marker ID: {ids[i]}")
                        print ("1차 방향 정렬 시작")
                        if 180 - abs(pitch) > 20 : 
                            if pitch > 0 : 
                                twist.angular.z = 0.2
                            else : 
                                twist.angular.z = -0.2
                        else :
                            if 180 - abs(pitch) > 4 :
                                if pitch > 0 : 
                                    twist.angular.z = 0.08
                                else : 
                                    twist.angular.z = -0.08
                            else : 
                                twist.angular.z = 0.0
                                print ("1차 직선 이동 시작")
                                twist.linear.x = 0.08
                if ids[i] == 4 and self.first_goal_reached and not self.rotation_in_progress:
                    if distance < 0.54:
                        print('도착')
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        # just once, move forward using wheel odom
                        self.move_forward_using_wheel_odom()
                    else :
                        print('2차 방향정렬 시작')
                        if 180 - abs(pitch) > 20 : 
                            print('절대값 비교시 20이상 차이남')
                            if pitch > 0 : 
                                twist.angular.z = 0.23
                            else : 
                                twist.angular.z = -0.2
                        else :
                            if 180 - abs(pitch) > 4 :
                                if pitch > 0 : 
                                    twist.angular.z = 0.1
                                else : 
                                    twist.angular.z = -0.08
                            else : 
                                twist.angular.z = 0.0
                                print('2차 직선 이동 시작')
                                twist.linear.x = 0.08
                            
 
                # self.process_docking(best_tvec, best_rvec, best_id, twist, angle_error)
        elif not self.rotation_in_progress and self.control_active:
            twist.linear.x = 0.0
            twist.angular.z = 0.18
            self.get_logger().info("Marker not found")
            # Reset error history when no marker is detected
            self.error_history = deque(maxlen=5)

        # Show the image for debugging
        cv2.imshow("ArUco Docking", frame)
        cv2.waitKey(1)

        # Update previous state values
        self.prev_angular_z = twist.angular.z
        self.prev_linear_x = twist.linear.x
        self.prev_angle_error = twist.angular.z  # update with current angular error

        print(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
        
        # Publish the command
        print("--------------------------------")
        print("PUBLISHING TWIST")
        print(f"twist.linear.x: {twist.linear.x:.2f}, twist.angular.z: {twist.angular.z:.2f}")
        print("--------------------------------")
        self.cmd_vel_pub.publish(twist)
        
        # 디버깅을 위한 상태 출력
        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"회전 상태: in_progress={self.rotation_in_progress}, first_goal={self.first_goal_reached}")
        
    def process_docking(self, tvec, rvec, marker_id, twist, angle_error):
        tvec = np.squeeze(tvec)
        rvec = np.rad2deg(np.squeeze(rvec))


        effective_Kp_y = self.Kp_y  # start with base gain
        # effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * trend)
        effective_Kp_y = self.Kp_y
        # effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * trend)
        effective_Kp_y = np.clip(effective_Kp_y, self.min_gain, self.max_gain)
        

        # Update error history with the new filtered angle
        # Calculate error trend (if sufficient history exists)
        if len(self.error_history) >= 2:
            trend = (self.error_history[-1] - self.error_history[0]) / (len(self.error_history) - 1)
        else:
            trend = 0.0


        # Adaptive gain adjustment based on error trend
        # effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * trend)

        derivative = (angle_error - self.prev_angle_error) / 0.1
        self.error_history.append(angle_error)

        diff_x = tvec[0] - self.target_tvec_x
        diff_z = tvec[2] - self.target_tvec_z
        diff_yaw = rvec[1] - self.target_rvec_y

        print(f"diff_x: {diff_x:.2f}, diff_z: {diff_z:.2f}, diff_yaw: {diff_yaw:.2f}")

        # 거리 기반 회전 오차
        # 거리가 가까울수록 회전 오차 허용 범위 줄임
        tolerance_yaw = 10.0
        if tvec[2] < 0.5:
            tolerance_yaw = 5.0
        elif tvec[2] < 0.3:
            tolerance_yaw = 3.0
        elif tvec[2] < 0.1:
            tolerance_yaw = 1.0


        # Determine angular control based on error magnitude
        if abs(angle_error) > tolerance_yaw:
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            # Slow down the forward speed
            # twist.linear.x *= 0.8
            print (f"VALUE: {effective_Kp_y * angle_error + derivative * self.kd_y}")
            angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.1), -0.1)
        else:
            if abs(tvec[0]) < self.tolerance_x and abs(tvec[2] - self.target_tvec_z) < self.tolerance_z:
                twist.linear.x = 0.0
                angular_control = 0.0
                print(f"✅ Docking complete (Marker ID: {marker_id})")
            else:
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                twist.linear.x = max(min(self.Kp_z * (tvec[2] - self.target_tvec_z), 0.2), -0.2)
                if abs(angle_error) > 15 and diff_z >= 0.8 :
                    # if angle_error > 0:
                    angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.2), -0.2)
                    # else : 
                        # angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.08), -0.08)
                else :                 
                    print (f"VALUE: {effective_Kp_y * angle_error + derivative * self.kd_y}")
                    angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.08), -0.08)
                # Enforce minimum angular speed if moving slowly
                if abs(twist.linear.x) < 0.08:
                    angular_control = 0.08 if angular_control > 0 else -0.08 

        twist.angular.z = angular_control

        # Log final control commands
        # print(f"Angle error after processing: {angle_error:.2f}")
        # print(f"Angular control (twist.angular.z): {angular_control:.3f}")
        
        # Update previous state values for the next iteration
        self.prev_angle_error = angle_error
        self.prev_mean_error = np.mean(self.error_history) if self.error_history else angle_error
        self.effective_Kp_y = effective_Kp_y
        self.prev_distance = diff_z
        

    def __del__(self):
        cv2.destroyAllWindows()

    def perform_90_degree_rotation(self):
        """서비스를 통해 90도 회전 요청"""
        # 회전 시작 플래그 설정
        self.rotation_in_progress = True
        
        request = Trigger.Request()
        future = self.rotate_client.call_async(request)
        self.get_logger().info("90도 회전 요청 전송")
        
        # 비동기 호출이므로 결과를 기다리지 않음
        # 회전이 완료되면 rotation_done 토픽을 통해 알림 받음

    def rotation_done_callback(self, msg):
        """회전 완료 토픽 콜백"""
        self.get_logger().info(f"회전 상태 메시지 수신: {msg.data}")
        
        if msg.data:  # True일 때 (회전 완료)
            self.get_logger().info("회전 완료 메시지 수신 - 회전 완료!")
            self.rotation_in_progress = False
            self.first_goal_reached = True
        else:  # False일 때 (회전 시작)
            self.get_logger().info("회전 시작 메시지 수신 - 회전 시작!")
            self.rotation_in_progress = True

def main(args=None):
    rp.init(args=args)
    node = ArUcoDockingController()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
