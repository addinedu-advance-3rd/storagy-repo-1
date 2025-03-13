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
import time
import multiprocessing
from http.server import HTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from fodDetection import start_fod_detector
import numpy as np
import socket

# ✅ 멀티프로세싱 공유 메모리 생성
manager = multiprocessing.Manager()
shared_data = manager.dict()
shared_data["distance"] = 0.0
shared_data["angle"] = 0.0

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


#지도 
class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        

        # 지도 파일 경로 설정
        self.map_yaml_path = os.path.join(os.path.dirname(__file__), 'room_x.yaml')
        self.get_logger().info(f"지도 YAML 파일 경로: {self.map_yaml_path}")
        
        # 지도 로드
        self.load_map()
        
        # 🔹 FOD 감지 마커 추가
        self.fod_marker = FODMarker(shared_data, self.resolution, self.origin)

        if self.fod_marker is None:
            print("❌ FODMarker가 초기화되지 않았습니다.")
        else:
            print("✅ FODMarker 초기화 완료!")

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
                            print(f"v10클라이언트로부터 메시지 수신: {message}")
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
        if not hasattr(self, 'map_image_color') or self.map_image_color is None:
            print("⚠️ 지도 이미지가 아직 초기화되지 않았습니다. 타이머 콜백 건너뜀.")
            return

        # 🔍 FODMarker가 생성되었는지 체크
        if not hasattr(self, 'fod_marker') or self.fod_marker is None:
            print("❌ FODMarker가 아직 생성되지 않음.")
            return

        # ✅ 지도 이미지 복사 (업데이트 전)
        display_img = self.map_image_color.copy()

        # 로봇 위치 업데이트 (self.latest_pose가 없으면 FOD 업데이트도 건너뜀)
        if self.latest_pose is None:
            print("⚠️ 아직 로봇 위치 데이터가 없습니다. 타이머 콜백 건너뜀.")
            return

        try:
            # ✅ 1️⃣ 로봇 위치 업데이트
            robot_x = self.latest_pose.position.x
            robot_y = self.latest_pose.position.y

            # 지도 메타데이터(원점, 해상도)를 이용해 픽셀 좌표로 변환
            pixel_x = int((robot_x - self.origin[0]) / self.resolution)
            pixel_y = display_img.shape[0] - int((robot_y - self.origin[1]) / self.resolution)

            # 로봇 위치 표시 (빨간 원 -> 로봇이미지로)
            # ✅ 1️⃣ 로봇 이미지(`robot.png`) 불러오기 및 크기 조정 (14x14px)
            # robot_img = cv2.imread("/home/addinedu/venv/develop/circle/src/circle/circle/robot.png", cv2.IMREAD_UNCHANGED)  # PNG 이미지 불러오기
            robot_img = cv2.imread("/home/addinedu/dev_ws/storagy-repo-1/circle/src/circle/circle/robot.png", cv2.IMREAD_UNCHANGED)  # PNG 이미지 불러오기

            if robot_img is None:
                print("❌ 로봇 이미지(robot.png) 로드 실패")
                return

            # ✅ PNG 이미지가 4채널(RGBA)이면 BGR로 변환
            if robot_img.shape[2] == 4:  # RGBA인지 확인
                robot_img = cv2.cvtColor(robot_img, cv2.COLOR_BGRA2BGR)  # 4채널 → 3채널 변환

            # ✅ 크기 조정
            robot_size = 16  # 기존 크기 (14x14)
            robot_img = cv2.resize(robot_img, (robot_size, robot_size))  # 크기 변경

            # ✅ 2️⃣ 지도 위에 로봇 이미지 붙이기 (이미지 모양 유지, 크기만 변경)
            y1, y2 = pixel_y - robot_size // 2, pixel_y + robot_size // 2
            x1, x2 = pixel_x - robot_size // 2, pixel_x + robot_size // 2

            # ✅ 지도 범위 체크 후 적용
            if 0 <= x1 and x2 < display_img.shape[1] and 0 <= y1 and y2 < display_img.shape[0]:
                display_img[y1:y2, x1:x2] = robot_img  # PNG에서 알파 채널 제거 후 정상적으로 덮어씌우기
            else:
                print("❌ 로봇 이미지가 지도 범위를 벗어남")

            cv2.putText(display_img, "Robot", (pixel_x + 10, pixel_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # ✅ 로봇의 yaw (방향) 계산
            yaw_deg = self.get_robot_yaw()

            # ✅ 로봇 진행 방향 표시 (화살표)
            arrow_length = 10
            arrow_x = int(pixel_x + arrow_length * math.cos(math.radians(yaw_deg)))
            arrow_y = int(pixel_y - arrow_length * math.sin(math.radians(yaw_deg)))
            cv2.arrowedLine(display_img, (pixel_x, pixel_y), (arrow_x, arrow_y), (255, 0, 0), 2)

            # ✅ 디버깅 정보 출력
            print(f"🟢 로봇 위치: x={robot_x:.2f}, y={robot_y:.2f}, yaw={yaw_deg:.2f}")
            print(f"🟢 픽셀 좌표 (OpenCV): ({pixel_x}, {pixel_y})")

        except Exception as e:
            print(f"❌ 로봇 위치 업데이트 중 오류 발생: {e}")
            return  # 로봇 위치 업데이트 실패 시 FOD 업데이트도 하지 않음

        # ✅ 2️⃣ 지도에 FOD 감지된 위치 업데이트 (이제 `robot_x`가 정의된 후 실행됨)
        try:
            distance_cm = shared_data.get("distance", 0.0)
            angle_deg = shared_data.get("angle", 0.0)

            if distance_cm > 0:
                self.fod_marker.update_fod_positions(robot_x, robot_y, yaw_deg)
                print(f"🟢 FOD 감지됨: 거리 {distance_cm:.2f} cm, 각도 {angle_deg:.2f}°")
            else:
                print("⚠️ 감지된 FOD 없음. 지도 업데이트 건너뜀.")

        except Exception as e:
            print(f"❌ FOD 업데이트 중 오류 발생: {e}")

        # ✅ 3️⃣ 지도에 FOD 표시
        try:
            if hasattr(self.fod_marker, 'draw_fod_on_map'):
                self.fod_marker.draw_fod_on_map(display_img)
            else:
                print("⚠️ FODMarker가 아직 초기화되지 않았습니다. 지도에 표시 불가.")
        except Exception as e:
            print(f"❌ FOD 지도 반영 중 오류 발생: {e}")

        # ✅ 4️⃣ 지도 업데이트 (웹 및 HTTP 서버 반영)
        try:
            MapHTTPHandler.map_image = display_img
            cv2.imshow("Map with Robot Position", display_img)
            cv2.waitKey(1)
        except Exception as e:
            print(f"❌ 지도 업데이트 중 오류 발생: {e}")



    def get_robot_yaw(self):
        """로봇의 orientation에서 yaw 각도를 계산하는 함수"""
        qx = self.latest_pose.orientation.x
        qy = self.latest_pose.orientation.y
        qz = self.latest_pose.orientation.z
        qw = self.latest_pose.orientation.w
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return math.degrees(yaw_rad)
    
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

#fod 마커
class FODMarker:
    def __init__(self, shared_data, resolution, origin):
        """ FOD 감지 데이터를 지도에 표시하는 클래스 """
        self.shared_data = shared_data  # 공유 데이터 (FOD 거리 및 각도)
        self.resolution = resolution  # 지도 해상도 (m/pixel)
        self.origin = origin  # 지도 원점 (실세계 좌표 기준)
        self.fod_position = None  # 감지된 첫 번째 FOD 위치 (튜플)
        self.fod_detected = False  # ✅ FOD가 지도에 표시되었는지 여부 (처음 표시 여부)

    def update_fod_positions(self, robot_x, robot_y, yaw_deg):
        """ FOD 감지 데이터를 기반으로 지도 좌표 계산 """
        if "distance" not in self.shared_data or "angle" not in self.shared_data:
            print("⚠️ FOD 데이터가 아직 없습니다. 업데이트 건너뜀.")
            return

        distance_cm = self.shared_data.get("distance", 0.0)  # 감지된 거리 (cm)
        angle_deg = self.shared_data.get("angle", 0.0)  # 감지된 상대각 (deg)

        if distance_cm == 0.0:
            print("⚠️ 감지된 FOD 없음. 지도 업데이트 건너뜀.")
            return  # 감지된 FOD 없음

        # 🔹 거리 단위 변환 (cm → m)
        distance_m = distance_cm / 150.0  # cm → m 변환

        # 🔹 로봇의 방향 (yaw) 및 FOD 상대각 변환
        yaw_rad = math.radians(yaw_deg)  # 로봇의 전역 yaw (radian)
        angle_rad = math.radians(angle_deg)  # FOD 상대 각도 (radian)

        # 🔹 FOD 실세계 좌표 계산 (로봇을 기준으로)
        total_angle = yaw_rad + angle_rad  # ✅ 상대각도를 고려한 총 방향
        fod_x = robot_x + distance_m * math.cos(total_angle)
        fod_y = robot_y + distance_m * math.sin(total_angle)

        # ✅ 디버깅 코드 추가 (실세계 좌표 확인)
        print(f"🟢 로봇 위치: ({robot_x:.2f}, {robot_y:.2f}), yaw={yaw_deg:.2f}°")
        print(f"🔎 변환된 FOD 실세계 좌표: ({fod_x:.2f}, {fod_y:.2f})")
        # 🔹 지도 픽셀 좌표 변환 (room_x.yaml 적용)
        fod_pixel_x = int((fod_x - self.origin[0]) / self.resolution)
        fod_pixel_y = int((fod_y - self.origin[1]) / self.resolution)


        # ✅ 디버깅 코드 추가 (픽셀 좌표 변환 과정 확인)
        print(f"🔎 지도 원점 (room_x.yaml): ({self.origin[0]}, {self.origin[1]})")
        print(f"🔎 변환된 FOD 픽셀 좌표: ({fod_pixel_x}, {fod_pixel_y})")

        # ✅ 지도 범위 내 확인
        height, width, _ = map_visualizer.map_image_color.shape
        if not (0 <= fod_pixel_x < width and 0 <= fod_pixel_y < height):
            print(f"❌ FOD 좌표가 지도 범위를 벗어남: ({fod_pixel_x}, {fod_pixel_y})")
            return  # 지도 범위를 벗어나면 표시하지 않음

        # ✅ 감지는 계속 수행하지만 지도에는 처음만 저장
        if not self.fod_detected:
            self.fod_position = (fod_pixel_x, fod_pixel_y)
            self.fod_detected = True  # ✅ 지도에 표시되었음을 기록
            print(f"🟢 FOD 최초 감지됨! 지도에 표시: {self.fod_position}")

    def draw_fod_on_map(self, map_image):
        """ 지도 위에 감지된 FOD를 시각적으로 표시 """
        if self.fod_position is None:
            return  # 지도에 표시할 FOD가 없음

        height, width, _ = map_image.shape  # 이미지 크기 가져오기

        x, y = self.fod_position

        # OpenCV는 (0,0)이 왼쪽 상단이므로 y 좌표 반전
        y = height - y

        # 지도 범위를 벗어나지 않도록 확인
        if 0 <= x < width and 0 <= y < height:
            cv2.circle(map_image, (x, y), 3, (0, 255, 0), -1)  # FOD 감지 위치 (초록색 원)
            cv2.putText(map_image, "FOD", (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 0, 255), 1)
        else:
            print(f"❌ FOD 좌표가 지도 범위를 벗어남: ({x}, {y})")



def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    
    # ✅ FOD 감지 프로세스 실행 추가 (shared_data 전달)
    print("✅ FOD 감지 프로세스 실행 시도...")
    
    fod_process = multiprocessing.Process(target=start_fod_detector, args=(shared_data,), daemon=True)
    fod_process.start()  # 프로세스 시작

    # 전역 변수로 노드 인스턴스 저장 (Flask 앱에서 접근할 수 있도록)
    global map_visualizer
    map_visualizer = node
    
    try:
        # ✅ 메인 ROS2 노드 실행
        rclpy.spin(node)
        print("ros2 실행 ")
    except KeyboardInterrupt:
        print("🛑 KeyboardInterrupt 발생, 종료 중...")
    except Exception as e:
        print(f"❌ ROS2 실행 중 예외 발생: {e}")
    finally:
        # ✅ ROS2 노드 종료
        node.get_logger().info("ROS2 노드 종료")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

        # ✅ FOD 감지 프로세스 종료
        print("🛑 FOD 감지 프로세스 종료 중...")
        fod_process.terminate()  # 프로세스 강제 종료
        fod_process.join()  # 프로세스가 완전히 종료될 때까지 대기
        print("✅ FOD 감지 프로세스 종료 완료")

        print("🚀 프로그램이 완전히 종료되었습니다.")


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