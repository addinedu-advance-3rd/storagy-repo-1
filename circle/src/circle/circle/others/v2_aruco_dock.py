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
# Simple 1D Kalman Filter for smoothing the angle measurement
class KalmanFilter:
    def __init__(self, initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0):
        self.x = initial_value         # state estimate
        self.P = error_estimate        # error covariance estimate
        self.Q = process_noise         # process noise covariance
        self.R = measurement_noise     # measurement noise covariance

    def update(self, measurement):
        # Kalman Gain
        K = self.P / (self.P + self.R)
        # Update estimate with measurement
        self.x = self.x + K * (measurement - self.x)
        # Update error covariance
        self.P = (1 - K) * self.P + self.Q
        return self.x

class ArUcoDockingController(Node):
    def __init__(self):
        super().__init__('aruco_dock')

        # ArUco marker settings
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # 50mm = 0.05m

        try:
            package_dir = get_package_share_directory('circle')
            calib_data_path = os.path.join(package_dir, 'vision/calib_data.npz')
            calib_data = np.load(calib_data_path)
        except FileNotFoundError as e:
            try:
                print(f"❌ File not found: {calib_data_path}")
                current_dir = os.path.dirname(os.path.abspath(__file__))
                calib_data_path = os.path.join(current_dir, "circle/src/circle/vision/calib_data.npz")
                calib_data = np.load(calib_data_path)
            except FileNotFoundError as e:
                raise FileNotFoundError(f"❌ File not found: {calib_data_path}")


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

        # Timer setup (20Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Minimum speed and PID gains
        self.MIN_LINEAR_SPEED = 0.03  # m/s
        self.MIN_ANGULAR_SPEED = 0.03  # rad/s

        # Docking target values and PID gains
        self.target_tvec_x = 0.00  # target alignment (0 for marker center)
        self.target_tvec_z = 0.01  # target distance (30cm)
        self.target_rvec_y = 0.00  # target rotation (maintain 0°)

        self.Kp_x = 0.1   # lateral control gain
        self.Kp_z = 0.4   # forward speed control gain
        self.Kp_y = 0.05  # angular control gain (base gain)

        # For storing previous control values
        self.prev_angular_z = 0.0
        self.prev_linear_x = 0.0 
        self.prev_angle_error = 0.0
        self.prev_mean_error = 0.0
        self.depth_in_use = False
        self.FINAL_DEPTH_STOP = 0.01  # final docking distance (1cm)

        # For filtering and trend estimation
        self.error_history = deque(maxlen=5)  # store last 5 filtered angle errors
        self.kalman_filter = KalmanFilter(initial_value=0.0, process_noise=1e-3, measurement_noise=1e-1, error_estimate=1.0)

        # Parameters for adaptive controller gain adjustment
        self.adaptive_alpha = 0.05  # scaling factor for the trend
        self.min_gain = 0.01       # minimum angular gain
        self.max_gain = 0.1        # maximum angular gain

        self.tolerance_x = 0.15
        self.tolerance_z = 0.01
        self.tolerance_yaw = 2.0


        self.get_logger().info('✅ ArUco docking controller activated.')
        print("✅ ArUco docking controller activated.", flush=True)

    def is_marker_detected(self):
        return self.control_active
    
    def service_callback(self, request, response):
        """Service callback for toggling docking activation."""
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
        """Compute the normalized angle error in the range [-180, 180]."""
        error = (current_angle - target_angle + 180) % 360 - 180
        return error

    def rotation_matrix_to_euler_angles(self, R_matrix):
        """
        Convert a rotation matrix to Euler angles using scipy.
        Returns: [roll, pitch, yaw] in degrees.
        """
        r = R.from_matrix(R_matrix)
        euler_angles = r.as_euler('xyz', degrees=True)
        return euler_angles

    def image_callback(self, msg):
        """Convert ROS image message to an OpenCV image."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        print("#########################")
        print("#########################")
        print("#########################")
        print("#########################")
        print("#########################")
        print("#########################")
        
        frame = self.rgb_image.copy()
        best_id = None
        angle_error = 0.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        twist = Twist()

        if ids is not None:
            self.get_logger().info("FOUND MARKER")
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            best_tvec = None
            best_rvec = None
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
                # Convert rotation vector to a rotation matrix and then to Euler angles (degrees)
                R_matrix, _ = cv2.Rodrigues(best_rvec)
                euler_angles_deg = self.rotation_matrix_to_euler_angles(R_matrix)
                
                # self.get_logger().info(f"Original rvec: {best_rvec.squeeze()}")
                # self.get_logger().info(f"Euler angles (deg): {euler_angles_deg}")
                
                # Use the pitch (euler_angles_deg[1]) for error calculation
                angle_error = self.normalized_angle_error(euler_angles_deg[2], 0.0)
                self.get_logger().info(f"Normalized angle error: {angle_error}")
                
                self.process_docking(best_tvec, best_rvec, best_id, twist, angle_error)

        else : 
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self.get_logger().info("마커 못찾음")
            #reset error history
            self.error_history = deque(maxlen=5)

        cv2.imshow("ArUco Docking", frame)
        self.prev_angular_z = twist.angular.z
        self.prev_linear_x = twist.linear.x
        self.prev_angle_error = angle_error
        print("진짜 publish 되는 값")
        print(f"twist.angular.z: {twist.angular.z}")
        print(f"twist.linear.x: {twist.linear.x}")
        print("진짜 publish 되는 값")
        
        
        #test
        #test
        #test
        #test
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # self.cmd_vel_pub.publish(twist)
        

        cv2.waitKey(1)


    def process_angle (self, angle_error):
        current_angle = ((angle_error + 180) % 360) - 180
        # ===== Filtering and Adaptive Gain Calculation =====
        # 1. Kalman Filter update for smoothing
        filtered_angle = self.kalman_filter.update(current_angle)


    def process_docking(self, tvec, rvec, marker_id, twist, angle_error):
        # Squeeze the translation and rotation vectors

        tvec = np.squeeze(tvec)
        rvec = np.rad2deg(np.squeeze(rvec))
        self.get_logger().info(f"Original rvec: {rvec}")
        self.get_logger().info(f"Original angle error: {angle_error}")

        # Normalize the angle error into the range [-180, 180]
        
        # 로봇이 움직이지 않을 때, 갑자기 오차가 커지는 것을 방지
        if hasattr(self, 'prev_angle_error'):
            angle_diff = abs(current_angle - self.prev_angle_error)
            if angle_diff > 30 and self.prev_linear_x == 0.0:
                current_angle = self.prev_angle_error
                self.get_logger().info("⚠️ Sudden angle change detected; retaining previous angle.")


        filtered_angle = self.process_angle(angle_error)
        self.error_history.append(filtered_angle)

        # 2. Append the filtered error into history for moving average & trend calculation
        # print(f"self.error_history: {self.error_history}")
        # moving_avg = np.mean(self.error_history) if self.error_history else filtered_angle
        # print(f"moving_avg: {moving_avg}, mean : {np.mean(self.error_history)}")
        # 3. Calculate error trend as the slope over the history window
        if len(self.error_history) >= 2:
            trend = (self.error_history[-1] - self.error_history[0]) / (len(self.error_history) - 1)
        else:
            trend = 0.0
        # 4. Adaptive gain adjustment based on the trend
        #    If error is increasing (positive trend), increase gain; if decreasing, reduce gain.
        # if trend > 0.0 :
        #     print("trend 가 양수이므로 각도 조정 강화") 
        #     effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * trend) * -0.8
        # effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * trend)
        effective_Kp_y = self.Kp_y * (1 + self.adaptive_alpha * filtered_angle)
        effective_Kp_y = np.clip(effective_Kp_y, self.min_gain, self.max_gain)


        # self.get_logger().info(f"Filtered angle: {filtered_angle:.2f}, Moving Average: {moving_avg:.2f}, Trend: {trend:.4f}")
        # self.get_logger().info(f"Adaptive Kp_y: {effective_Kp_y:.4f}")
        # ====================================================

        # tolerance_x = 0.15
        # tolerance_z = 0.01
        # tolerance_yaw = 2.0

        # Adaptive angular control using the filtered angle error and effective gain
        if abs(filtered_angle) > self.tolerance_yaw:
            print ("각도 범위가 TOL 보다 큼")
            # 감속
            twist.linear.x *= 0.8
            angular_control = max(min(effective_Kp_y * filtered_angle * 0.5, 0.08), -0.08)
        
            self.get_logger().info(f"🔄 Aligning angle... Current filtered angle: {filtered_angle:.2f}")
        else:
            print("angle error 가 TOL 보다 작음")
            if abs(tvec[0]) < self.tolerance_x and abs(tvec[2] - self.target_tvec_z) < self.tolerance_z:
                twist.linear.x = 0.0
                angular_control = 0.0
                self.get_logger().info(f"✅ Docking complete (Marker ID: {marker_id})")
            else:
                print ("각도는 맞음")
                twist.linear.x = max(min(self.Kp_z * (tvec[2] - self.target_tvec_z), 0.2), -0.2)
                self.get_logger().info(f"twist.linear.x: {twist.linear.x}")
                angular_control = max(min(effective_Kp_y * filtered_angle * 0.3, 0.05), -0.05)
                if abs(twist.linear.x) < 0.08:
                    angular_control = 0.08 if angular_control > 0 else -0.08

        # Exponential smoothing of angular control to avoid abrupt changes
        # if hasattr(self, 'prev_angular_z'):
            # angular_control = 0.7 * self.prev_angular_z + 0.3 * angular_control

        # 이전 평균 오차와 현재 평균 오차의 차이가 0.01 이상이면 오차가 커졌다고 판단
        # if abs(self.prev_mean_error - moving_avg) > 0.1:
        #     self.get_logger().info("⚠️ Error has increased; reducing linear speed.")
        #     twist.linear.x *= 0.8
        #     twist.angular.z *= -0.8

        # 이전 평균 오차와 현재 평균 오차의 차이가 0.01 이상이면 오차가 커졌다고 판단
        # if abs(self.prev_angle_error) - abs(filtered_angle) < -10:
        #     self.get_logger().info("⚠️ Error has increased; reducing linear speed.")
        #     twist.linear.x *= 0.8
        #     twist.angular.z *= -1.1
        # elif abs(self.prev_angle_error) - abs(filtered_angle) > 10:
        #     self.get_logger().info("⚠️ Error has decreased; increasing linear speed.")
        #     # twist.linear.x 
        #     twist.angular.z *= 0.9
        

        twist.angular.z = angular_control
        # if filtered_angle < 0 : 
        #     twist.angular.z = abs(twist.angular.z) * -1

        # if abs(twist.linear.x) < 0.08 and twist.linear.x != 0.0:
        #     twist.linear.x = 0.08 if twist.linear.x > 0 else -0.08
            
        # if abs(twist.angular.z) < 0.08 and twist.angular.z != 0.0:
        #     twist.angular.z = 0.08 if twist.angular.z > 0 else -0.08


        # Store the current filtered angle for next iteration
        self.prev_angle_error = filtered_angle


        self.prev_mean_error = moving_avg


        self.get_logger().info(f"Filtered angle after processing: {filtered_angle:.2f}")
        self.get_logger().info(f"Angular control (twist.angular.z): {angular_control:.3f}")
        self.get_logger().info(f"Final twist.linear.x: {twist.linear.x:.3f}, twist.angular.z: {twist.angular.z:.3f}")


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
