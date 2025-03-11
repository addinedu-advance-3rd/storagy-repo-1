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

from package_msg.action import ActionMessages #-- 수정 필요
from rclpy.action import ActionClient

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

        # ---✅ xArm 액션 클라이언트 생성---
        self.action_client= ActionClient(self, ActionMessages, 'action_messages',)

        # ArUco marker settings
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # marker size in meters
        self.effective_Kp_y = 0.0
        
        # Load calibration data

        calib_data = np.load('/home/storagy/storagy-repo-1/circle/src/circle/circle/calib_data.npz')
        # try:
        #     package_dir = get_package_share_directory('circle')
        #     # calib_data_path = os.path.join(package_dir, 'vision/calib_data.npz')
        #     calib_data = np.load(calib_data_path)
        # except FileNotFoundError as e:
        #     try:
        #         self.get_logger().info(f"❌ File not found: {calib_data_path}")
        #         current_dir = os.path.dirname(os.path.abspath(__file__))
        #         calib_data_path = os.path.join(current_dir, "circle/src/circle/vision/calib_data.npz")
        #         calib_data = np.load(calib_data_path)
        #     except FileNotFoundError as e:
        #         raise FileNotFoundError(f"❌ File not found: {calib_data_path}")

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
        self.service = self.create_service(Trigger, 'dock_robot', self.service_callback)

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
        # For filtering and trend estimation
        self.error_history = deque(maxlen=5)
        self.kalman_filter = KalmanFilter(initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0)

        # Parameters for adaptive controller gain adjustment
        self.adaptive_alpha = 0.05  # scaling factor for the trend
        self.min_gain = 0.01       # minimum angular gain
        self.max_gain = 0.1        # maximum angular gain

        # Tolerances
        self.tolerance_x = 0.15
        self.tolerance_z = 0.01
        # self.tolerance_yaw = 10.0

        print('✅ ArUco docking controller activated.')

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

    def timer_callback(self):
        frame = self.rgb_image.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        twist = Twist()

        if ids is not None:
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
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.cmtx, self.dist)
                frame = cv2.drawFrameAxes(frame, self.cmtx, self.dist, rvec, tvec, 0.05)
                distance = np.linalg.norm(tvec)
                if distance < min_distance:
                    min_distance = distance
                    best_tvec = tvec
                    best_rvec = rvec
                    best_id = ids[i][0]

            if best_tvec is not None:
                # Convert rotation vector to rotation matrix and then to Euler angles (in degrees)
                R_matrix, _ = cv2.Rodrigues(best_rvec)
                euler_angles_deg = self.rotation_matrix_to_euler_angles(R_matrix)
                
                # Use the pitch (index 1) for error calculation
                print(f"euler_angles_deg: {euler_angles_deg[2]:.2f}")
                angle_error = (euler_angles_deg[2]+90)
                angle_error = self.normalized_angle_error(angle_error, 0.0)
                print (f"Normalized angle error: {angle_error:.2f}")

                

                # print (f"Euler angles: {euler_angles_deg[0]:.2f}, {euler_angles_deg[1]:.2f}, {euler_angles_deg[2]:.2f}")
                
                self.process_docking(best_tvec, best_rvec, best_id, twist, angle_error)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.2
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
        self.cmd_vel_pub.publish(twist)

        # ---client 동작 관련---
    def send_xarm_command(self, command):
        """XArm에게 동작 명령을 보내는 함수"""
        goal_msg = ActionMessages.Goal()
        goal_msg.command = command

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('action_messages 서비스를 기다리는 중...')

        self.get_logger().info(f"📡 xArm에 '{command}' 명령 전송")

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)  # ✅ 콜백 등록

    def goal_response_callback(self, future):
        """xArm이 명령을 수락했는지 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("🚫 xArm 작업이 거부되었습니다")
            return
        self.get_logger().info("✅ xArm 작업이 시작되었습니다!")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)  # ✅ 결과 콜백 등록
    
    def get_result_callback(self, future):
        """xArm 동작 결과를 받는 콜백"""
        result = future.result()
        self.get_logger().info(f"🔔 xArm 작업 완료: {result.result}")
        # ----------------------
        
    def process_docking(self, tvec, rvec, marker_id, twist, angle_error):
        self.send_xarm_command("도착")


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


        self.send_xarm_command("도착")

        # Determine angular control based on error magnitude
        if abs(angle_error) > tolerance_yaw:  # 회전 오차가 허용 범위 초과
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            self.get_logger().info("Angle error exceeds tolerance, performing angular correction.")
            # Slow down the forward speed
            # twist.linear.x *= 0.8
            print (f"VALUE: {effective_Kp_y * angle_error + derivative * self.kd_y}")
            
            angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.1), -0.1)
            
        else:   # 회전 오차가 허용 범위 이내
            if abs(tvec[0]) < self.tolerance_x and abs(tvec[2] - self.target_tvec_z) < self.tolerance_z:
                twist.linear.x = 0.0
                angular_control = 0.0
                print(f"✅ Docking complete (Marker ID: {marker_id})")

                # --- ✅ 도킹 완료 후 xArm에 "move_home" 명령 전송 ---
                self.send_xarm_command("도착")
            else:
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                print("Within angular tolerance; applying distance control.")
                twist.linear.x = max(min(self.Kp_z * (tvec[2] - self.target_tvec_z), 0.2), -0.2)
                if abs(angle_error) > 15 and diff_z >= 0.8 :    # 회전 오차와 거리 오차가 모두 클 때
                    # if angle_error > 0:
                    angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.2), -0.2)
                    # else : 
                        # angular_control = max(min(effective_Kp_y * angle_error + derivative * self.kd_y, 0.08), -0.08)
                else :  # 거리와 각도가 근소한 오차일 때                 
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

def main(args=None):
    rp.init(args=args)
    node = ArUcoDockingController()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
