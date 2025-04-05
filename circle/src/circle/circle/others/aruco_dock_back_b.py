import cv2
import numpy as np
import rclpy as rp
import math
import os
import cv2.aruco as aruco
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from collections import deque

# --- CONFIGURATION ---
# Physical marker size in meters (e.g., 5 cm marker)

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
        self.marker_length = 0.05  
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # marker size in meters
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        try:
            package_dir = get_package_share_directory('circle')
            calib_data_path = os.path.join(package_dir, 'calib_data.npz')
            calib_data = np.load(calib_data_path)
        except FileNotFoundError as e:
            try:
                self.get_logger().info(f"❌ File not found: {calib_data_path}")
                current_dir = os.path.dirname(os.path.abspath(__file__))
                calib_data_path = os.path.join(current_dir, "calib_data.npz")
                calib_data = np.load(calib_data_path)
            except FileNotFoundError as e:
                raise FileNotFoundError(f"❌ File not found: {calib_data_path}")
        self.cmtx = calib_data['camMatrix']  # camera matrix
        self.dist = calib_data['distCoeff']    # distortion coefficients

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_active = False
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
        # For filtering and trend estimation
        self.error_history = deque(maxlen=5)
        self.kalman_filter = KalmanFilter(initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0)

        # Parameters for adaptive controller gain adjustment
        self.adaptive_alpha = 0.05  # scaling factor for the trend

    def service_callback(self, request, response):
        """Toggle docking activation via service call."""
        self.control_active = not self.control_active
        if self.control_active:
            self.get_logger().info("🚀 ArUco docking activated.")
            response.success = True
            response.message = "ArUco docking activated."
        else:
            self.get_logger().info("🛑 ArUco docking deactivated.")
            response.success = True
            response.message = "ArUco docking deactivated."
        return response
    
    def reset_state(self):
        """Reset docking state."""
        self.prev_angular_z = 0.0
        self.control_active = False
        self.get_logger().info("🔄 Docking state reset.")


    def image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')


    def timer_callback(self):
        if not self.control_active:
            return
        
        frame = self.rgb_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        twist = Twist()

        if ids is not None:
            self.get_logger().info("마커 발견")
            
            # 마커 그리기
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 각 마커의 포즈 추정
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.cmtx, self.dist)
            
            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                # 마커 좌표축 그리기
                aruco.drawAxis(frame, self.cmtx, self.dist, rvec, tvec, self.marker_length * 0.6)
                
                # 카메라에서 마커까지의 거리 계산
                distance = np.linalg.norm(tvec)
                
                # 도킹 각도 계산 (도 단위)
                x = tvec[0][0]  # 수평 오프셋
                z = tvec[0][2]  # 전방 거리
                angle = math.degrees(math.atan2(x, z))
                
                # 필터링된 각도 계산
                self.filtered_angle = self.kalman_filter.update(angle)
                
                # 각도 변화율 계산 (미분)
                angle_rate = self.filtered_angle - self.prev_filtered_angle
                self.prev_filtered_angle = self.filtered_angle
                
                # 제어 신호 계산
                # 1. 각도 제어 (회전)
                angular_z = -self.Kp_y * self.filtered_angle - self.kd_y * angle_rate
                
                # 2. 거리 제어 (전진)
                linear_x = self.Kp_z * (distance - self.target_tvec_z)
                
                # 3. 측면 정렬 제어 (측면 이동)
                linear_y = self.Kp_x * (x - self.target_tvec_x)
                
                # 최소 속도 적용
                if abs(linear_x) < self.MIN_LINEAR_SPEED:
                    linear_x = 0.0
                
                if abs(angular_z) < self.MIN_ANGULAR_SPEED:
                    angular_z = 0.0
                
                # 제어 명령 적용
                twist.linear.x = linear_x
                twist.linear.y = linear_y
                twist.angular.z = angular_z
                
                # 로그 출력
                self.get_logger().info(f"마커 ID: {ids[i][0]}, 거리: {distance:.2f}m, 각도: {angle:.1f}°")
                self.get_logger().info(f"제어 명령 - 선속도: {linear_x:.2f}m/s, 각속도: {angular_z:.2f}rad/s")
                
                # 프레임에 정보 표시
                cv2.putText(frame, f"거리: {distance:.2f}m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"각도: {angle:.1f}°", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 제어 명령 발행
            self.cmd_vel_pub.publish(twist)
        else:
            # 마커가 보이지 않을 때 로봇 정지
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("마커가 보이지 않습니다. 로봇 정지.")
        
        # 영상 표시 (디버깅용)
def main(args=None):
    rp.init(args=args)
    aruco_docking_controller = ArUcoDockingController()
    rp.spin(aruco_docking_controller)
    aruco_docking_controller.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()