import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from package_msg.action import ActionMessages
import numpy as np
import cv2
from ultralytics import YOLO
from xarm.wrapper import XArmAPI
import time

class XArmActionServer(Node):
    def __init__(self):
        super().__init__('xarm_control')

        # 액션 서버 초기화
        self.action_server = ActionServer(
            self,
            ActionMessages,
            'action_messages',
            self.execute_callback
        )

        # 저장된 카메라 캘리브레이션 정보 로드
        self.camera_matrix = np.load("/home/addinedu/camera_matrix.npy")
        self.dist_coeffs = np.load("/home/addinedu/dist_coeffs.npy")
        self.homography_matrix = np.load("/home/addinedu/homography_matrix2.npy")

        # 로봇팔 초기화
        self.arm = XArmAPI("192.168.1.182")
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        # YOLO 모델 로드
        self.model = YOLO("/home/addinedu/tools_ver3/weights/best.pt")
        self.cap = cv2.VideoCapture(0)

        # 감지할 공구 리스트 (실제 모델 클래스명 확인 후 수정 필요!)
        self.tool_priority = ["Driver", "Spanner", "Hammer"]

        # 도구를 내려놓을 좌표 (각각 도구에 대응)
        self.drop_positions = {
            "Driver": (-123, -320, 330, 180, 0, 90),
            "Spanner": (-14, -320, 330, 180, 0, 90),
            "Hammer": (111.2, -320, 330, 180, 0, 90)
        }

        # 홈 포지션 정의
        self.home_position = (-8, -174, 450, 180, 0, 90)
        self.tool_positions = {}

    def execute_callback(self, goal_handle):
        """'도착' 키워드를 받으면 로봇 동작 실행 → 완료 후 '완료' 반환"""
        self.get_logger().info('📩 "도착" 신호 수신, 작업 시작!')

        # 피드백을 클라이언트에 전송 (작업 중임을 알림)
        feedback_msg = ActionMessages.Feedback()
        #self.get_logger().info(f"🧐 디버깅: {dir(feedback_msg)}")

        feedback_msg.feedback = "작업을 수행 중.."
        goal_handle.publish_feedback(feedback_msg)

        # 도구 감지
        self.detect_tools()
        if not self.tool_positions:
            self.get_logger().warn("⚠️ 도구 감지 실패!")
            goal_handle.abort()
            return ActionMessages.Result(result="도구 감지 실패")
        
        for tool in self.tool_priority:
            if tool not in self.tool_positions:
                self.get_logger().warn(f"⚠️ {tool} 위치를 찾을 수 없습니다! 건너뜁니다.")
                continue  # 도구 감지 실패 시 건너뜀

            # 픽셀 좌표 → 로봇 좌표 변환
            pick_coords = list(self.pixel_to_robot(*self.tool_positions[tool][:2]))  # 리스트 변환
            angle = self.tool_positions[tool][2]  # 추출된 각도 (degree 단위)

            if angle is not None:
                # 각도를 라디안으로 변환
                angle_rad = np.radians(angle)

                # x, y 좌표 보정
                pick_coords[0] += 140 * np.cos(angle_rad)  
                pick_coords[1] += 140 * np.sin(angle_rad)

            # ✅ 올바른 위치에서 호출 (모든 도구 이동)
            self.move_to_tool(tool, pick_coords, angle)

        self.get_logger().info("🔄 모든 작업 완료! 홈 포지션으로 복귀 중...")
        self.arm.set_position(*self.home_position, speed=200, mvacc=100, wait=True)  # 모든 작업 후 홈 포지션 복귀

        # ✅ 작업 완료 후 '완료' 키워드 반환
        goal_handle.succeed()
        return ActionMessages.Result(result="완료")
        
    def pixel_to_robot(self, x, y, fixed_z=450, roll=180, pitch=0, yaw=90):
        """픽셀 좌표를 로봇 좌표로 변환"""
        src_pixel = np.array([[x, y]], dtype=np.float32)
        dst_pixel = cv2.undistortPoints(src_pixel, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        src = np.array([[dst_pixel[0][0][0], dst_pixel[0][0][1], 1]], dtype=np.float32).T
        dst = np.dot(self.homography_matrix, src)
        dst /= dst[2]
        return (dst[0].item(), dst[1].item(), fixed_z, roll, pitch, yaw)

    def estimate_angle_pca(self, mask):
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        points = np.column_stack(np.where(mask > 0))
        if len(points) < 10:
            return None, None
        mean, eigenvectors = cv2.PCACompute(points.astype(np.float32), mean=None)
        if mean is None or eigenvectors is None or len(eigenvectors) == 0:
            return None, None
        principal_axis = eigenvectors[0]
        #angle = np.arctan2(principal_axis[1], principal_axis[0]) * (180 / np.pi) + 90
        angle = np.arctan2(principal_axis[1], principal_axis[0]) * (180 / np.pi)
        angle = (angle + 360) % 180 # 기준을 0도로 설정
        if angle < 90:
            angle = angle + 90
        else:
            angle = angle - 90

        return angle, tuple(mean.flatten().astype(int))

    def detect_tools(self):
        """YOLO 모델을 사용하여 도구 감지"""
        self.tool_positions.clear()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ 카메라 영상을 가져올 수 없습니다!")
            return
        
        detect_results = self.model(frame, verbose=False)
        for results in detect_results:
            if results.masks is not None:
                for mask, box, cls in zip(results.masks.data.cpu().numpy(), results.boxes.xyxy.cpu().numpy(), results.boxes.cls.cpu().numpy()):
                    class_name = self.model.names[int(cls)]
                    if class_name in self.tool_priority:
                        mask = (mask * 255).astype(np.uint8)
                        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        for cnt in contours:
                            if cv2.contourArea(cnt) > 500:
                                M = cv2.moments(cnt)
                                if M["m00"] != 0:
                                    cx = int(M["m10"] / M["m00"])
                                    cy = int(M["m01"] / M["m00"])
                                    bbox_cx = int((box[0] + box[2]) / 2)
                                    bbox_cy = int((box[1] + box[3]) / 2)
                                    final_cx = int((cx + bbox_cx) / 2)
                                    final_cy = int((cy + bbox_cy) / 2)
                                    angle, _ = self.estimate_angle_pca(mask)
                                    if angle is not None:
                                        self.tool_positions[class_name] = (final_cx, final_cy, angle)

        self.get_logger().info(f"🔍 감지된 도구: {self.tool_positions}")
        
    def move_to_tool(self, tool_name, pick_coords, angle):
        """로봇팔을 이동하여 도구를 집고 지정된 위치에 놓음"""
        self.get_logger().info(f"🚀 {tool_name} 위치로 이동: {pick_coords}, 회전 각도: {angle:.2f}°")
        
        # 공구 잡기 위해 움직이는 과정
        self.arm.close_lite6_gripper() 
        self.arm.set_position(*pick_coords, speed=200, mvacc=100, wait=True)
        self.arm.set_position(pick_coords[0], pick_coords[1], pick_coords[2], pick_coords[3], pick_coords[4], angle, speed=200, mvacc=100, wait=True)
        time.sleep(1)
        self.arm.set_position(pick_coords[0], pick_coords[1], 320, pick_coords[3], pick_coords[4], angle, speed=200, mvacc=100, wait=True)
        
        # 공구 잡기
        self.arm.open_lite6_gripper()
        time.sleep(1)

        # 공구 잡은 후 들어올리기
        self.arm.set_position(*pick_coords, speed=200, mvacc=100, wait=True)
        
        # 공구 정리하기 위해 움직이는 과정
        drop_coords = self.drop_positions[tool_name]
        self.get_logger().info(f"📍 {tool_name}을(를) {drop_coords} 위치로 이동")
        self.arm.set_position(drop_coords[0], drop_coords[1], 450, drop_coords[3], drop_coords[4], drop_coords[5], speed=200, mvacc=100, wait=True)
        self.arm.set_position(*drop_coords, speed=200, mvacc=100, wait=True)
        
        # 공구 놓기
        self.arm.close_lite6_gripper()
        time.sleep(1)

        # 공구 놓은 후 
        self.arm.set_position(drop_coords[0], drop_coords[1], 450, drop_coords[3], drop_coords[4], drop_coords[5], speed=200, mvacc=100, wait=True)

    def destroy_node(self):
        """노드 종료 시 실행되는 함수"""
        self.get_logger().info("🛑 서버 종료 중... 카메라 종료")
        
        if self.cap.isOpened():
            self.cap.release()  # ✅ 카메라 해제
            self.get_logger().info("📸 카메라가 정상적으로 종료되었습니다.")
        
        cv2.destroyAllWindows()  # ✅ 모든 OpenCV 창 닫기
        super().destroy_node()

def main(args = None):
    rclpy.init(args=args)
    node = XArmActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

