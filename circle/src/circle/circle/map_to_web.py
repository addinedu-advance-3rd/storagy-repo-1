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
import numpy as np
import socket

# 웹소켓 클라이언트 연결을 저장할 세트
connected_clients = set()

# Flask-SocketIO 통합을 위한 콜백 함수
robot_pose_callback = None

# 전역 변수로 웹소켓 이벤트 루프 저장
websocket_loop = None

class MapHTTPHandler(BaseHTTPRequestHandler):
    map_image = None  # 지도 이미지를 저장할 클래스 변수
    
    def log_message(self, format, *args):
        # 로깅 메시지 출력
        print(format % args)
    
    def do_GET(self):
        try:
            if self.path == '/':
                # HTML 페이지 제공
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.send_header('Access-Control-Allow-Origin', '*')
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
                    self.send_header('Content-type', 'text/plain')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(b'Map image not available')
            else:
                self.send_response(404)
                self.send_header('Content-type', 'text/plain')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'Not found')
        except Exception as e:
            print(f"HTTP 핸들러 오류: {e}")
            # 오류 발생 시 500 응답
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(f"Server error: {str(e)}".encode('utf-8'))

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        
        # 지도 파일 경로 설정
        self.map_yaml_path = os.path.join(os.path.dirname(__file__), 'room_11.yaml')
        self.get_logger().info(f"지도 YAML 파일 경로: {self.map_yaml_path}")
        
        # 지도 로드
        self.load_map()
        
        # 로봇 위치 구독
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.latest_pose = None
        
        # 타이머 설정 (10Hz로 지도 업데이트)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # HTTP 서버 시작 (포트 8000)
        self.start_http_server(port=8000)
        
        # 웹소켓 서버 시작
        self.start_websocket_server()
        
        # Flask-SocketIO 콜백 초기화
        self.flask_socketio_callback = None

    def load_map(self):
        """YAML 파일에서 지도 정보를 로드하고 이미지를 준비합니다."""
        try:
            # YAML 파일 로드
            with open(self.map_yaml_path, 'r') as file:
                map_data = yaml.safe_load(file)
            
            # 지도 이미지 파일 경로 (YAML 파일 위치 기준 상대 경로)
            map_image_path = os.path.join(os.path.dirname(self.map_yaml_path), map_data['image'])
            self.get_logger().info(f"지도 이미지 파일 경로: {map_image_path}")
            
            # 지도 메타데이터 저장
            self.resolution = map_data['resolution']  # 미터/픽셀
            self.origin = map_data['origin']  # [x, y, theta] 형식, 미터 단위
            
            # 지도 이미지 로드 (그레이스케일)
            self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
            if self.map_image is None:
                self.get_logger().error(f"지도 이미지를 로드할 수 없습니다: {map_image_path}")
                self._create_default_map()
                return
            
            # 지도 이미지 크기 출력
            height, width = self.map_image.shape
            self.get_logger().info(f"지도 이미지 크기: {width}x{height} 픽셀")
            self.get_logger().info(f"지도 해상도: {self.resolution} 미터/픽셀")
            self.get_logger().info(f"지도 원점: {self.origin}")
            
            # 컬러 이미지로 변환 (시각화용)
            self.map_image_color = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
            
            # 지도 이미지 반전 (흰색 배경, 검은색 장애물)
            _, self.map_image_binary = cv2.threshold(self.map_image, 127, 255, cv2.THRESH_BINARY_INV)
            
            # HTTP 핸들러에 지도 이미지 설정
            MapHTTPHandler.map_image = self.map_image_color
            
            self.get_logger().info("지도 로드 완료")
        except Exception as e:
            self.get_logger().error(f"지도 로드 중 오류 발생: {e}")
            self._create_default_map()

    def _create_default_map(self):
        """기본 지도 이미지를 생성합니다."""
        self.get_logger().info("기본 지도 이미지 생성")
        
        # 기본 지도 크기 및 메타데이터 설정
        width, height = 800, 800
        self.resolution = 0.05  # 미터/픽셀
        self.origin = [-20.0, -20.0, 0.0]  # [x, y, theta] 형식, 미터 단위
        
        # 흰색 배경의 기본 지도 이미지 생성
        self.map_image = np.ones((height, width), dtype=np.uint8) * 255
        
        # 테두리 그리기
        border_width = 20
        self.map_image[0:border_width, :] = 0  # 상단 테두리
        self.map_image[-border_width:, :] = 0  # 하단 테두리
        self.map_image[:, 0:border_width] = 0  # 좌측 테두리
        self.map_image[:, -border_width:] = 0  # 우측 테두리
        
        # 중앙에 텍스트 추가를 위한 컬러 이미지 변환
        self.map_image_color = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
        
        # 중앙에 텍스트 추가
        text = "지도 파일을 찾을 수 없습니다"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        cv2.putText(self.map_image_color, text, (text_x, text_y), font, font_scale, (0, 0, 255), font_thickness)
        
        # 지도 이미지 반전 (흰색 배경, 검은색 장애물)
        _, self.map_image_binary = cv2.threshold(self.map_image, 127, 255, cv2.THRESH_BINARY_INV)
        
        # HTTP 핸들러에 지도 이미지 설정
        MapHTTPHandler.map_image = self.map_image_color
        
        self.get_logger().info(f"기본 지도 이미지 생성 완료: {width}x{height} 픽셀")

    def start_websocket_server(self, port=8765):
        # 웹소켓 서버 시작 함수
        def run_server():
            # 새 이벤트 루프 생성
            global websocket_loop
            websocket_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(websocket_loop)
            
            # 웹소켓 라이브러리 버전 확인
            import pkg_resources
            try:
                ws_version = pkg_resources.get_distribution("websockets").version
                major_version = int(ws_version.split('.')[0])
                print(f"웹소켓 라이브러리 버전: {ws_version}")
            except Exception as e:
                print(f"버전 확인 오류: {e}")
                major_version = 0  # 기본값
            
            # 웹소켓 핸들러 정의 - 버전 10 이상용
            async def handler_v10(websocket):
                # 클라이언트 연결 시 저장
                connected_clients.add(websocket)
                print(f"웹소켓 클라이언트 연결됨: {websocket.remote_address}")
                
                try:
                    # 연결 유지를 위한 간단한 메시지 처리 루프
                    while True:
                        try:
                            # 클라이언트로부터 메시지 수신 (타임아웃 설정)
                            message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                            print(f"클라이언트로부터 메시지 수신: {message}")
                        except asyncio.TimeoutError:
                            # 타임아웃은 정상적인 상황이므로 계속 진행
                            pass
                        except websockets.exceptions.ConnectionClosed:
                            # 연결이 닫힌 경우 루프 종료
                            print(f"클라이언트 연결 종료: {websocket.remote_address}")
                            break
                except Exception as e:
                    print(f"웹소켓 오류: {e}")
                finally:
                    try:
                        connected_clients.remove(websocket)
                    except:
                        pass
                    print(f"웹소켓 클라이언트 연결 종료: {websocket.remote_address}")
            
            # 웹소켓 핸들러 정의 - 버전 10 미만용
            async def handler_legacy(websocket, path):
                # 클라이언트 연결 시 저장
                connected_clients.add(websocket)
                print(f"웹소켓 클라이언트 연결됨: {websocket.remote_address}")
                
                try:
                    # 연결 유지를 위한 간단한 메시지 처리 루프
                    while True:
                        try:
                            # 클라이언트로부터 메시지 수신 (타임아웃 설정)
                            message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                            print(f"클라이언트로부터 메시지 수신: {message}")
                        except asyncio.TimeoutError:
                            # 타임아웃은 정상적인 상황이므로 계속 진행
                            pass
                        except websockets.exceptions.ConnectionClosed:
                            # 연결이 닫힌 경우 루프 종료
                            print(f"클라이언트 연결 종료: {websocket.remote_address}")
                            break
                except Exception as e:
                    print(f"웹소켓 오류: {e}")
                finally:
                    try:
                        connected_clients.remove(websocket)
                    except:
                        pass
                    print(f"웹소켓 클라이언트 연결 종료: {websocket.remote_address}")
            
            # 웹소켓 서버 시작
            async def start_server():
                if major_version >= 10:
                    # 버전 10 이상
                    print("웹소켓 버전 10 이상 핸들러 사용")
                    async with websockets.serve(handler_v10, "0.0.0.0", port):
                        print(f"웹소켓 서버 시작됨 - ws://0.0.0.0:{port}")
                        await asyncio.Future()
                else:
                    # 버전 10 미만
                    print("웹소켓 버전 10 미만 핸들러 사용")
                    async with websockets.serve(handler_legacy, "0.0.0.0", port):
                        print(f"웹소켓 서버 시작됨 - ws://0.0.0.0:{port}")
                        await asyncio.Future()
            
            try:
                websocket_loop.run_until_complete(start_server())
            except KeyboardInterrupt:
                print("웹소켓 서버 종료 중...")
            except Exception as e:
                print(f"웹소켓 서버 오류: {e}")
            finally:
                websocket_loop.close()
        
        # 별도 스레드에서 웹소켓 서버 실행
        self.websocket_thread = threading.Thread(
            target=run_server,
            daemon=True
        )
        self.websocket_thread.start()
        self.get_logger().info(f"웹소켓 서버가 ws://0.0.0.0:{port} 에서 시작되었습니다.")

    def start_http_server(self, port=8000):
        # HTTP 서버 시작 함수
        def run_server():
            server_address = ('0.0.0.0', port)  # 모든 인터페이스에서 접속 허용
            httpd = HTTPServer(server_address, MapHTTPHandler)
            self.get_logger().info(f"HTTP 서버가 http://0.0.0.0:{port} 에서 시작되었습니다.")
            print(f"외부 접속 URL: http://{self._get_ip_address()}:{port}")
            httpd.serve_forever()
        
        # 별도 스레드에서 HTTP 서버 실행
        self.http_thread = threading.Thread(
            target=run_server,
            daemon=True
        )
        self.http_thread.start()

    def _get_ip_address(self):
        """서버의 IP 주소를 반환합니다."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # 구글 DNS에 연결하여 자신의 IP 확인 (실제 연결은 하지 않음)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = "127.0.0.1"
        finally:
            s.close()
        return ip

    def pose_callback(self, msg):
        # 로봇 위치 메시지 수신 시 처리
        self.latest_pose = msg.pose.pose
        
        # 지도 이미지가 없으면 콜백 무시
        if not hasattr(self, 'map_image_color'):
            return
        
        # 로봇의 월드 좌표 (map 프레임, m 단위)
        robot_x = self.latest_pose.position.x
        robot_y = self.latest_pose.position.y
        
        # 디버깅 정보 출력
        self.get_logger().debug(f"로봇 위치 수신: x={robot_x:.2f}, y={robot_y:.2f}")

    def timer_callback(self):
        # 지도 이미지가 없으면 타이머 콜백 무시
        if not hasattr(self, 'map_image_color'):
            return
        
        # 매번 원본 지도 이미지를 복사하여 업데이트
        display_img = self.map_image_color.copy()  # 컬러 이미지 사용
        
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
            # 웹 페이지에서 사용할 픽셀 좌표 계산
            # 원본 이미지 기준 좌표 (y축은 위에서 아래로 증가)
            web_pixel_x = pixel_x
            web_pixel_y = display_img.shape[0] - int((robot_y - self.origin[1]) / self.resolution)
            
            # 이미지 크기 정보 추가
            img_width = display_img.shape[1]
            img_height = display_img.shape[0]
            
            # 디버깅 정보 출력
            print(f"로봇 위치 (미터): ({robot_x:.2f}, {robot_y:.2f})")
            print(f"원점 (미터): ({self.origin[0]:.2f}, {self.origin[1]:.2f})")
            print(f"해상도 (미터/픽셀): {self.resolution:.6f}")
            print(f"이미지 크기 (픽셀): {img_width}x{img_height}")
            print(f"픽셀 좌표 (OpenCV): ({pixel_x}, {pixel_y})")
            print(f"픽셀 좌표 (웹): ({web_pixel_x}, {web_pixel_y})")
            
            # 이미지 크기 정보도 함께 전송
            self.send_robot_pose(robot_x, robot_y, web_pixel_x, web_pixel_y, yaw_deg, img_width, img_height)
        
        # 업데이트된 이미지 창 표시
        cv2.imshow("Map with Robot Position", display_img)
        cv2.waitKey(1)
        
        # HTTP 핸들러에 업데이트된 이미지 설정
        MapHTTPHandler.map_image = display_img
    
    def send_robot_pose(self, robot_x, robot_y, pixel_x, pixel_y, yaw_deg, img_width, img_height):
        # 웹소켓으로 로봇 위치 데이터 전송
        if not connected_clients:
            print("연결된 웹소켓 클라이언트가 없습니다.")
            return
        
        # 전송할 데이터 준비
        data = {
            'type': 'robot_pose',
            'robot_x': robot_x,
            'robot_y': robot_y,
            'pixel_x': pixel_x,
            'pixel_y': pixel_y,
            'yaw_deg': yaw_deg,
            'img_width': img_width,
            'img_height': img_height
        }
        
        # 모든 클라이언트에 메시지 전송
        message = json.dumps(data)
        print(f"웹소켓 메시지 전송 (클라이언트 수: {len(connected_clients)})")
        
        # 웹소켓 메시지 전송 - 간소화된 방식
        global websocket_loop
        if websocket_loop and websocket_loop.is_running():
            for client in list(connected_clients):
                try:
                    # 비동기 전송을 동기적으로 처리
                    asyncio.run_coroutine_threadsafe(
                        client.send(message),
                        websocket_loop
                    )
                except Exception as e:
                    print(f"웹소켓 메시지 전송 오류: {e}")
                    # 오류 발생 시 클라이언트 연결 제거 시도
                    try:
                        connected_clients.discard(client)
                    except:
                        pass
        else:
            print("웹소켓 이벤트 루프가 실행 중이 아닙니다.")

    # Flask-SocketIO 콜백 설정 메서드 추가
    def set_flask_socketio_callback(self, callback):
        self.flask_socketio_callback = callback

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    
    # 전역 변수로 노드 인스턴스 저장 (Flask 앱에서 접근할 수 있도록)
    global map_visualizer
    map_visualizer = node
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

# 전역 변수로 MapVisualizer 인스턴스 저장
map_visualizer = None

# Flask 앱에서 호출할 함수 - 콜백 설정
def set_robot_pose_callback(callback):
    global map_visualizer
    if map_visualizer:
        map_visualizer.set_flask_socketio_callback(callback)
        return True
    return False

if __name__ == '__main__':
    main()