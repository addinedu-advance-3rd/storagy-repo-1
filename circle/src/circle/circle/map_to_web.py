import os
import cv2
import yaml
import rclpy
import math
import json
import asyncio
import threading
import websockets
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

# 웹소켓 클라이언트 연결을 저장할 세트
connected_clients = set()

class MapHTTPHandler(BaseHTTPRequestHandler):
    map_image = None  # 지도 이미지를 저장할 클래스 변수
    
    def do_GET(self):
        # CORS 헤더 추가
        self.send_header('Access-Control-Allow-Origin', '*')
        
        if self.path == '/':
            # HTML 페이지 제공
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            # HTML 파일 읽기
            html_path = os.path.join(os.path.dirname(__file__), 'map.html')
            with open(html_path, 'rb') as file:
                self.wfile.write(file.read())
                
        elif self.path == '/map-image':
            # 지도 이미지 제공
            if MapHTTPHandler.map_image is not None:
                self.send_response(200)
                self.send_header('Content-type', 'image/png')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                
                # OpenCV 이미지를 PNG로 인코딩하여 전송
                _, img_encoded = cv2.imencode('.png', MapHTTPHandler.map_image)
                self.wfile.write(img_encoded.tobytes())
            else:
                self.send_response(404)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
        else:
            self.send_response(404)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        # 1. map.yaml 파일 로드 및 메타데이터 추출
        yaml_file_path = "/home/storagy/circle/room_11.yaml"
        with open(yaml_file_path, 'r') as f:
            self.map_meta = yaml.safe_load(f)
        self.resolution = self.map_meta['resolution']     # m/pixel
        self.origin = self.map_meta['origin']             # [origin_x, origin_y, theta]
        
        # 2. 지도 이미지 로드 (절대 경로로)
        map_dir = os.path.dirname(yaml_file_path)
        map_file = os.path.join(map_dir, self.map_meta['image'])
        self.map_image_original = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if self.map_image_original is None:
            raise FileNotFoundError(f"Map image file not found: {map_file}")
        
        # 지도 이미지를 3채널로 변환 (웹 표시용)
        self.map_image_color = cv2.cvtColor(self.map_image_original, cv2.COLOR_GRAY2BGR)
        
        # HTTP 핸들러에 지도 이미지 설정
        MapHTTPHandler.map_image = self.map_image_color
        
        # 초기 복사본 (매번 원본 위에 그리기 위함)
        self.map_image = self.map_image_original.copy()
        
        # 3. /amcl_pose 토픽 구독 (메시지 타입: PoseWithCovarianceStamped)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # 4. Timer callback 등록 (0.5초마다 지도 업데이트)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # 최신 Pose 저장 변수 (초기값 None)
        self.latest_pose = None
        
        # 웹소켓 서버 시작
        self.start_websocket_server()
        
        # HTTP 서버 시작
        self.start_http_server()

    def start_websocket_server(self):
        # 웹소켓 서버 시작 함수
        async def websocket_handler(websocket, path):
            # 클라이언트 연결 시 저장
            connected_clients.add(websocket)
            try:
                await websocket.wait_closed()
            finally:
                connected_clients.remove(websocket)
        
        async def start_server():
            server = await websockets.serve(websocket_handler, "0.0.0.0", 8765)
            await server.wait_closed()
        
        # 비동기 이벤트 루프 생성 및 웹소켓 서버 시작
        self.loop = asyncio.new_event_loop()
        self.websocket_thread = threading.Thread(
            target=self._run_websocket_server,
            args=(self.loop, start_server),
            daemon=True
        )
        self.websocket_thread.start()
    
    def _run_websocket_server(self, loop, coro):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(coro())
    
    def start_http_server(self):
        # HTTP 서버 시작 함수
        self.http_server = HTTPServer(('0.0.0.0', 8080), MapHTTPHandler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True
        )
        self.http_thread.start()
        self.get_logger().info("HTTP 서버가 http://localhost:8080/ 에서 시작되었습니다.")

    def pose_callback(self, msg):
        # /amcl_pose 메시지에서 Pose (geometry_msgs/Pose)를 저장
        self.latest_pose = msg.pose.pose
        self.get_logger().info(
            f"Received pose: x={self.latest_pose.position.x:.2f}, y={self.latest_pose.position.y:.2f}"
        )

    def timer_callback(self):
        # 매번 원본 지도 이미지를 복사하여 업데이트
        display_img = self.map_image_original.copy()
        
        if self.latest_pose is not None:
            # 로봇의 월드 좌표 (map 프레임, m 단위)
            robot_x = self.latest_pose.position.x
            robot_y = self.latest_pose.position.y

            # 지도 메타데이터(원점, 해상도)를 이용해 픽셀 좌표로 변환
            pixel_x = int((robot_x - self.origin[0]) / self.resolution)
            # OpenCV에서는 y 좌표가 위에서 아래로 증가하므로 이미지 높이에서 뺌
            pixel_y = display_img.shape[0] - int((robot_y - self.origin[1]) / self.resolution)
            
            # 로봇 위치 표시 (빨간 원)
            cv2.circle(display_img, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
            cv2.putText(display_img, "Robot", (pixel_x + 10, pixel_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 로봇의 orientation (Quaternion → yaw 변환)
            qx = self.latest_pose.orientation.x
            qy = self.latest_pose.orientation.y
            qz = self.latest_pose.orientation.z
            qw = self.latest_pose.orientation.w
            yaw_rad = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            yaw_deg = math.degrees(yaw_rad)
            
            # 화살표 끝 점 계산 (로봇의 진행 방향 표시)
            arrow_length = 20  # 픽셀 단위
            arrow_x = int(pixel_x + arrow_length * math.cos(yaw_rad))
            arrow_y = int(pixel_y - arrow_length * math.sin(yaw_rad))
            cv2.arrowedLine(display_img, (pixel_x, pixel_y), (arrow_x, arrow_y), (255, 0, 0), 2)
            
            # 웹소켓으로 로봇 위치 데이터 전송
            self.send_robot_pose(robot_x, robot_y, pixel_x, pixel_y, yaw_deg)
        
        # 업데이트된 이미지 창 표시
        cv2.imshow("Map with Robot Position", display_img)
        cv2.waitKey(1)
    
    def send_robot_pose(self, robot_x, robot_y, pixel_x, pixel_y, yaw_deg):
        # 웹소켓으로 로봇 위치 데이터 전송
        if not connected_clients:
            return
        
        # 전송할 데이터 준비
        data = {
            'type': 'robot_pose',
            'robot_x': robot_x,
            'robot_y': robot_y,
            'pixel_x': pixel_x,
            'pixel_y': pixel_y,
            'yaw_deg': yaw_deg
        }
        
        # 비동기 이벤트 루프에 전송 작업 추가
        asyncio.run_coroutine_threadsafe(self._send_to_all_clients(json.dumps(data)), self.loop)
    
    async def _send_to_all_clients(self, message):
        # 모든 연결된 클라이언트에 메시지 전송
        if connected_clients:
            await asyncio.gather(
                *[client.send(message) for client in connected_clients],
                return_exceptions=True
            )

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
